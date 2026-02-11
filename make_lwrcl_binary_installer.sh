#!/bin/bash
CURRENT=`pwd`

BINARY_PATH=$1
if [[ ! -f "$BINARY_PATH" ]]; then
    echo "Please provide a valid binary path."
    exit 1
fi

FILE_NAME=$(basename "$BINARY_PATH")

DEP_DIR=$(mktemp -d)
mkdir -p "$DEP_DIR/bin"
mkdir -p "$DEP_DIR/lib"
mkdir -p "$DEP_DIR/etc"

cp "$BINARY_PATH" "$DEP_DIR/bin/"
DIR_PATH=$(dirname "$BINARY_PATH")
cp -rf "$DIR_PATH/config" "$DEP_DIR/bin/"

ldd "$BINARY_PATH" | awk '/=>/ {print $3}' | grep -v '\\(' | while read lib; do
    cp "$lib" "$DEP_DIR/lib/"
done

cd "$DEP_DIR"
tar czf ../package.tar.gz .
cd ..

INSTALL_FILES=()
while IFS= read -r -d $'\0' file; do
    base_name=$(basename "$file")
    if [[ "$base_name" != "." && "$base_name" != ".." ]]; then
        INSTALL_FILES+=("$base_name")
    fi
done < <(find "$DEP_DIR/bin" -mindepth 1 -maxdepth 1 -print0)

LIB_FILES=()
while IFS= read -r -d $'\0' file; do
    base_name=$(basename "$file")
    if [[ "$base_name" != "." && "$base_name" != ".." ]]; then
        LIB_FILES+=("$base_name")
    fi
done < <(find "$DEP_DIR/lib" -mindepth 1 -maxdepth 1 -print0)

cat > installer_script.sh << 'EOF'
#!/bin/bash
declare -a BIN_FILES=()
declare -a LIB_FILES=()

EOF

echo 'BIN_FILES=(' >> installer_script.sh
for file in "${INSTALL_FILES[@]}"; do
    echo "    \"$file\"" >> installer_script.sh
done
echo ')' >> installer_script.sh

echo 'LIB_FILES=(' >> installer_script.sh
for file in "${LIB_FILES[@]}"; do
    echo "    \"$file\"" >> installer_script.sh
done
echo ')' >> installer_script.sh


cat >> installer_script.sh << 'EOF'
function install() {
    mkdir -p /opt/lwrcl/bin
    mkdir -p /opt/lwrcl/lib
    TMP_DIR=$(mktemp -d)
    ARCHIVE_OFFSET=$(awk '/^__ARCHIVE_BELOW__/ {print NR + 1; exit 0; }' "$0")
    cp $0 $TMP_DIR
    cd $TMP_DIR
    tail -n +$ARCHIVE_OFFSET $0 | tar xzf -
    for file in "${BIN_FILES[@]}"; do
        sudo cp -r "$TMP_DIR/bin/$file" /opt/lwrcl/bin/
    done
    for file in "${LIB_FILES[@]}"; do
        sudo cp -r "$TMP_DIR/lib/$file" /opt/lwrcl/lib/
    done
    sudo ldconfig
    rm -rf "$TMP_DIR"
    echo "Installation completed."
}

function uninstall() {
    for file in "${BIN_FILES[@]}"; do
        sudo rm -rf "/opt/lwrcl/bin/$file"
    done
    for file in "${LIB_FILES[@]}"; do
        sudo rm -rf "/opt/lwrcl/lib/$file"
    done
    sudo ldconfig
    echo "Uninstallation completed."
}

case "$1" in
    install)
        install
        ;;
    uninstall)
        uninstall
        ;;
    *)
        echo "Usage: $0 {install|uninstall}"
        exit 1
        ;;
esac

exit 0
EOF

echo '__ARCHIVE_BELOW__' >> installer_script.sh
cat package.tar.gz >> installer_script.sh
chmod +x installer_script.sh
mv /tmp/installer_script.sh ${CURRENT}/${FILE_NAME}_installer

echo "Self-extracting installer has been created."
echo "Transfer this installer to the target machine and execute it to install the binary."
echo "Run ./${FILE_NAME}_installer install to install the binary."

# Cleanup
rm -rf "$DEP_DIR" package.tar.gz

