#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

camel_to_snake() {
    local input="$1"
    echo "$input" | sed -E 's/([A-Z]+)([A-Z][a-z])/\1_\2/g' | sed -E 's/([a-z\d])([A-Z])/\1_\2/g' | tr '[:upper:]' '[:lower:]'
}

snake_to_camel() {
    local input="$1"
    awk -v value="$input" 'BEGIN {
        n = split(value, parts, "_")
        for (i = 1; i <= n; i++) {
            if (parts[i] == "") {
                continue
            }
            printf "%s%s", toupper(substr(parts[i], 1, 1)), substr(parts[i], 2)
        }
        printf "\n"
    }'
}

folder_path="$(pwd)"
echo "Start processing in $folder_path"

find "$folder_path" -type f -name "*.hpp" | while read -r file_path; do
    echo "Found $file_path"
    # cycloneファイルはスキップ
    if [[ "$file_path" == *"_cyclone.hpp"* ]]; then
        echo "Skipping cyclone file: $file_path"
        continue
    fi

    file_name=$(basename "$file_path")
    class_name=$(basename "$file_name" .hpp)
    directory_path=$(dirname "$file_path")

    # snake_caseファイルはスキップ（idlcは常にPascalCaseを生成するため）
    if [[ ! "$class_name" =~ [A-Z] ]]; then
        echo "Skipping already snake_case file: $file_name"
        continue
    fi
    
    # 元ファイルを_cyclone.hppにリネーム
    cyclone_file_path="${file_path%.hpp}_cyclone.hpp"
    echo "Renaming $file_path to $cyclone_file_path"
    mv "$file_path" "$cyclone_file_path"
    
    # リネーム後のファイルの内容を読み込み
    file_content=$(cat "$cyclone_file_path")

    # 既に修正済みかチェック（enable_shared_from_thisまたはSharedPtrが存在するか）
    if echo "$file_content" | grep -q "enable_shared_from_this\|SharedPtr"; then
        echo "The file $file_name has already been modified. Skipping..."
        continue
    fi

    # classキーワードが存在するかチェック（より柔軟なパターンマッチング）
    if ! echo "$file_content" | grep -qE "^[[:space:]]*class[[:space:]]+$class_name[[:space:]]*$|^[[:space:]]*class[[:space:]]+$class_name[[:space:]]*\{"; then
        echo "No class definition found for $class_name in $file_name. Skipping..."
        continue
    fi

    echo "Processing $file_name for class $class_name"

    # <memory>インクルードを追加（既に存在しない場合のみ）
    if ! echo "$file_content" | grep -q "#include <memory>"; then
        # Insert portably; BSD sed and GNU sed disagree on the append syntax.
        if echo "$file_content" | grep -q "#ifndef.*_HPP"; then
            file_content=$(printf '%s\n' "$file_content" | awk '
                /^#define.*_HPP/ && !inserted {
                    print
                    print ""
                    print "#include <memory>"
                    inserted=1
                    next
                }
                { print }
            ')
        else
            file_content=$(printf '%s\n' "$file_content" | awk '
                BEGIN { print "#include <memory>" }
                { print }
            ')
        fi
        echo "Added #include <memory> to $file_name"
    fi

    # 他のIDL生成ヘッダーファイルのインクルードを_cyclone.hpp付きに変更
    # #include <memory>より後、クラス定義より前にある.hppファイルのインクルードを対象とする
    echo "Updating other IDL header includes in $file_name"
    
    # 一時ファイルを使用してより安全に処理
    temp_file=$(mktemp)
    memory_found=false
    class_found=false
    
    while IFS= read -r line; do
        if [[ "$line" =~ ^[[:space:]]*#include[[:space:]]+\<memory\> ]]; then
            memory_found=true
            echo "$line" >> "$temp_file"
        elif [[ "$line" =~ ^[[:space:]]*class[[:space:]]+$class_name ]]; then
            class_found=true
            echo "$line" >> "$temp_file"
        elif [[ "$memory_found" == "true" && "$class_found" == "false" && "$line" =~ ^[[:space:]]*#include[[:space:]]+\"([^\"]+)\.hpp\".*$ ]]; then
            # <memory>より後、クラス定義より前の.hppインクルードを_cyclone.hpp付きに変更
            header_name="${BASH_REMATCH[1]}"
            # 対応する_cyclone.hppファイルが存在するかチェック
            cyclone_header_path="$directory_path/${header_name}_cyclone.hpp"
            if [ -f "$cyclone_header_path" ] || [[ "$header_name" != *"_cyclone" ]]; then
                echo "#include \"${header_name}_cyclone.hpp\"" >> "$temp_file"
                echo "  Updated include: ${header_name}.hpp -> ${header_name}_cyclone.hpp"
            else
                echo "$line" >> "$temp_file"
            fi
        else
            echo "$line" >> "$temp_file"
        fi
    done <<< "$file_content"
    
    file_content=$(cat "$temp_file")
    rm -f "$temp_file"

    # class定義にstd::enable_shared_from_thisを追加（より柔軟なパターンマッチング）
    modified_content=$(echo "$file_content" | sed -E "s/^([[:space:]]*)class[[:space:]]+$class_name([[:space:]]*$|[[:space:]]*\{)/\1class $class_name : public std::enable_shared_from_this<$class_name>\2/")
    
    # 変更が実際に行われたかチェック
    if [ "$file_content" = "$modified_content" ]; then
        echo "Failed to modify class definition for $class_name. Trying alternative pattern..."
        # 代替パターン: クラス名の後に何らかの文字がある場合
        modified_content=$(echo "$file_content" | sed "s/class $class_name/class $class_name : public std::enable_shared_from_this<$class_name>/")
    fi

    # public:セクションの直後にSharedPtr型定義を追加
    # まずSharedPtrを追加
    modified_content_with_insert1=$(echo "$modified_content" | awk -v class_name="$class_name" -v insert_code="    using SharedPtr = std::shared_ptr<${class_name}>;" '
    /^class '"$class_name"'/ {in_class=1}
    in_class && /^public:/ && !shared_ptr_added {
        print
        print insert_code
        shared_ptr_added=1
        next
    }
    {print}
    ')

    # 次にConstSharedPtrを追加
    modified_content_with_insert2=$(echo "$modified_content_with_insert1" | awk -v class_name="$class_name" -v insert_code="    using ConstSharedPtr = std::shared_ptr<const ${class_name}>;" '
    /^class '"$class_name"'/ {in_class=1}
    in_class && /using SharedPtr/ && !const_shared_ptr_added {
        print
        print insert_code
        const_shared_ptr_added=1
        next
    }
    {print}
    ')

    # 修正内容をcycloneファイルに書き戻し
    echo "$modified_content_with_insert2" > "$cyclone_file_path"
    
    # snake_caseファイルの生成
    snake_name=$(camel_to_snake "$class_name")
    directory_name=$(basename "$directory_path")
    parent_path=$(dirname "$directory_path")
    parent_directory_path=$(basename "$parent_path")
    
    snake_file_path="$directory_path/${snake_name}.hpp"
    include_guard=$(echo "${parent_directory_path}__${directory_name}__${snake_name}.hpp" | tr '[:lower:]' '[:upper:]' | tr '/' '_' | tr '.' '_')

    # snake_caseファイルを作成
    {
    echo "#ifndef ${include_guard}"
    echo "#define ${include_guard}"
    echo ""
    echo "#include \"dds/dds.hpp\""
    echo "#include \"${class_name}_cyclone.hpp\""
    echo ""
    echo "#endif  // ${include_guard}"
    } > "$snake_file_path"
    echo "Created snake_case file: ${snake_name}.hpp"

    echo "Modified the file $file_name"
done

# ----------------------------------------------------------------
# Fix for case-insensitive filesystems (e.g., macOS mounted volumes)
# On case-insensitive FS, idlc may overwrite snake_case wrappers
# when regenerating PascalCase files (Image.hpp == image.hpp).
# This pass ensures all wrappers contain the correct wrapper content.
# ----------------------------------------------------------------
echo "Ensuring snake_case wrappers are correct..."
find "$folder_path" -type f -name "*_cyclone.hpp" | while read -r cyclone_file; do
    base_name=$(basename "$cyclone_file" "_cyclone.hpp")
    directory_path=$(dirname "$cyclone_file")

    # Skip files that don't have an uppercase letter (shouldn't happen, but guard)
    if [[ ! "$base_name" =~ [A-Z] ]]; then
        continue
    fi

    snake_name=$(camel_to_snake "$base_name")
    snake_file="$directory_path/${snake_name}.hpp"

    # If the snake_case file is identical to the PascalCase name, skip
    # (e.g., a single-lowercase-word class — unlikely but safe)
    if [ "$snake_name" = "$base_name" ]; then
        continue
    fi

    directory_name=$(basename "$directory_path")
    parent_path=$(dirname "$directory_path")
    parent_directory_path=$(basename "$parent_path")
    include_guard=$(echo "${parent_directory_path}__${directory_name}__${snake_name}.hpp" | tr '[:lower:]' '[:upper:]' | tr '/' '_' | tr '.' '_')

    # (Re)create the wrapper — idempotent
    {
    echo "#ifndef ${include_guard}"
    echo "#define ${include_guard}"
    echo ""
    echo "#include \"dds/dds.hpp\""
    echo "#include \"${base_name}_cyclone.hpp\""
    echo ""
    echo "#endif  // ${include_guard}"
    } > "$snake_file"
    echo "Ensured wrapper: ${snake_name}.hpp -> ${base_name}_cyclone.hpp"
done

# ----------------------------------------------------------------
# Normalize generated-IDL includes across package/subdir boundaries.
# idlc can emit includes such as "action_msgs/msg/GoalInfo.hpp".
# After this script renames generated headers to *_cyclone.hpp, those
# includes must point at the renamed headers, especially on case-sensitive
# include resolution during macOS builds.
# ----------------------------------------------------------------
echo "Updating generated header includes across packages..."
generated_root=$(dirname "$folder_path")
find "$folder_path" -type f -name "*_cyclone.hpp" | while read -r cyclone_file; do
    temp_file=$(mktemp)
    changed=false
    directory_path=$(dirname "$cyclone_file")

    while IFS= read -r line; do
        if [[ "$line" =~ ^([[:space:]]*#include[[:space:]]+\")([^\"]+/)?([^/\"\<]+)\.hpp(\".*)$ ]]; then
            include_prefix="${BASH_REMATCH[1]}"
            include_dir="${BASH_REMATCH[2]}"
            header_name="${BASH_REMATCH[3]}"
            include_suffix="${BASH_REMATCH[4]}"

            if [[ "$header_name" != *_cyclone ]]; then
                if [ -n "$include_dir" ]; then
                    candidate="${generated_root}/${include_dir}${header_name}_cyclone.hpp"
                else
                    candidate="${directory_path}/${header_name}_cyclone.hpp"
                fi

                if [ -f "$candidate" ]; then
                    echo "${include_prefix}${include_dir}${header_name}_cyclone.hpp${include_suffix}" >> "$temp_file"
                    echo "  Updated generated include in $(basename "$cyclone_file"): ${include_dir}${header_name}.hpp -> ${include_dir}${header_name}_cyclone.hpp"
                    changed=true
                    continue
                fi
            fi
        fi

        echo "$line" >> "$temp_file"
    done < "$cyclone_file"

    if [ "$changed" = "true" ]; then
        mv "$temp_file" "$cyclone_file"
    else
        rm -f "$temp_file"
    fi
done

# サービス用のリクエスト/レスポンス処理
echo "Creating service files..."
find "$folder_path" -type f -name "*_Request_cyclone.hpp" | while read -r request_file_path; do
    echo "Found request cyclone file: $request_file_path"
    
    # _request_cyclone.hppを_response_cyclone.hppに置き換えたファイルパスを生成
    response_file_path="${request_file_path/_Request_cyclone.hpp/_Response_cyclone.hpp}"
    
    # 対応する_response_cyclone.hppファイルが存在するかチェック
    if [ -f "$response_file_path" ]; then
        echo "Corresponding response cyclone file exists: $response_file_path"
        
        # ベース名を生成（_request_cyclone.hppから_request_cyclone.hppを除去）
        base_path="${request_file_path%_Request_cyclone.hpp}"
        base_name=$(basename "$base_path")
        
        # サービス名を抽出（例：Empty_Request -> Empty）
        service_name="${base_name%_Request}"
        
        directory_path=$(dirname "$request_file_path")
        directory_name=$(basename "$directory_path")
        parent_path=$(dirname "$directory_path")
        parent_directory_path=$(basename "$parent_path")
        
        # snake_caseに変換
        snake_name=$(camel_to_snake "$service_name")
        service_file_path="$directory_path/${snake_name}.hpp"
        
        # 既存ファイルが存在する場合はスキップ
        if [ -f "$service_file_path" ]; then
            echo "Service file already exists: $service_file_path. Skipping..."
            continue
        fi

        # 新しいベース名のファイルを作成
        touch "$service_file_path"
        
        include_guard=$(echo "${parent_directory_path}__${directory_name}__${snake_name}.hpp" | tr '[:lower:]' '[:upper:]' | tr '/' '_' | tr '.' '_')
        {
        echo -e "#ifndef ${include_guard}"
        echo -e "#define ${include_guard}\n"
        echo -e "#include \"${service_name}_Request_cyclone.hpp\"\n"
        echo -e "#include \"${service_name}_Response_cyclone.hpp\"\n"
        echo -e "namespace ${parent_directory_path} {\n"
        echo -e "  namespace ${directory_name} {\n"
        echo -e "    struct ${service_name} {\n"
        echo -e "      using Request = ${parent_directory_path}::${directory_name}::${service_name}_Request;"
        echo -e "      using Response = ${parent_directory_path}::${directory_name}::${service_name}_Response;"
        echo -e "    };\n"
        echo -e "  };\n"
        echo -e "};\n"
        echo -e "#endif  // ${include_guard}"
        } > "$service_file_path"
        echo "Created service file: ${snake_name}.hpp"
    else
        echo "Corresponding response cyclone file does not exist for: $request_file_path"
    fi
done

echo "CycloneDDS C++ processing completed."

# .cppファイルのインクルードを修正
echo "Updating .cpp file includes..."
find "$folder_path" -type f -name "*.cpp" | while read -r cpp_file; do
    echo "Checking cpp file: $cpp_file"
    
    # .cppファイルの内容を読み込み
    cpp_content=$(cat "$cpp_file")
    
    # 対応する.hppファイルが存在する場合のみインクルードを変更
    # 例: #include "Duration.hpp" -> #include "Duration_cyclone.hpp"
    modified_cpp_content="$cpp_content"
    
    # インクルード行を一つずつ処理
    while IFS= read -r line; do
        if [[ "$line" =~ ^[[:space:]]*#include[[:space:]]+\"([^\"]+)\.hpp\".*$ ]]; then
            header_name="${BASH_REMATCH[1]}"
            original_header_path=$(dirname "$cpp_file")/"${header_name}.hpp"
            cyclone_header_path=$(dirname "$cpp_file")/"${header_name}_cyclone.hpp"
            
            # 対応する_cyclone.hppファイルが存在するかチェック
            if [ -f "$cyclone_header_path" ]; then
                echo "  Found cyclone header: ${header_name}_cyclone.hpp"
                # インクルードを置換
                modified_cpp_content=$(echo "$modified_cpp_content" | sed "s|#include \"${header_name}\.hpp\"|#include \"${header_name}_cyclone.hpp\"|g")
                echo "  Updated include: ${header_name}.hpp -> ${header_name}_cyclone.hpp"
            fi
        fi
    done <<< "$cpp_content"
    
    # 変更があった場合のみファイルを更新
    if [ "$cpp_content" != "$modified_cpp_content" ]; then
        echo "$modified_cpp_content" > "$cpp_file"
        echo "  Successfully updated: $(basename "$cpp_file")"
    else
        echo "  No changes needed in: $(basename "$cpp_file")"
    fi
done

echo "Cpp file include updates completed."

# ----------------------------------------------------------------
# Expose IDL-generated members as ROS 2-style public fields.
# Removes "()" accessors and renames <name>_ -> <name>.
# ----------------------------------------------------------------
if [ -f "$SCRIPT_DIR/expose_idl_members.py" ]; then
    PYTHON3_BIN="$(command -v python3 || command -v python || true)"
    if [ -n "$PYTHON3_BIN" ]; then
        echo "Running expose_idl_members.py (cyclonedds) on $folder_path"
        "$PYTHON3_BIN" "$SCRIPT_DIR/expose_idl_members.py" --mode cyclonedds "$folder_path"
    else
        echo "WARN: python3 not found; skipping expose_idl_members.py" >&2
    fi
fi
