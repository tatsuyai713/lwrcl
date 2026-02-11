import 'dart:ffi';
import 'dart:io';

import 'package:ffi/ffi.dart';

DynamicLibrary openLwrclLibrary({String? overridePath}) {
  if (overridePath != null && overridePath.isNotEmpty) {
    return DynamicLibrary.open(overridePath);
  }
  final envPath = Platform.environment['LWRCL_FFI_LIB'];
  if (envPath != null && envPath.isNotEmpty) {
    return DynamicLibrary.open(envPath);
  }

  if (Platform.isMacOS) {
    return DynamicLibrary.open('liblwrcl_ffi.dylib');
  }
  if (Platform.isWindows) {
    return DynamicLibrary.open('lwrcl_ffi.dll');
  }
  return DynamicLibrary.open('liblwrcl_ffi.so');
}

class LwrclBindings {
  LwrclBindings(DynamicLibrary library)
      : _library = library,
        lwrclInit = library.lookupFunction<Void Function(), void Function()>(
          'lwrcl_ffi_init',
        ),
        lwrclInitArgs = library.lookupFunction<
            Void Function(Int32, Pointer<Pointer<Utf8>>),
            void Function(int, Pointer<Pointer<Utf8>>)>(
          'lwrcl_ffi_init_args',
        ),
        lwrclShutdown = library.lookupFunction<Void Function(), void Function()>(
          'lwrcl_ffi_shutdown',
        ),
        lwrclOk = library.lookupFunction<Int32 Function(), int Function()>(
          'lwrcl_ffi_ok',
        ),
        nodeCreate = library.lookupFunction<
            Pointer<Void> Function(Pointer<Utf8>, Pointer<Utf8>),
            Pointer<Void> Function(Pointer<Utf8>, Pointer<Utf8>)>(
          'lwrcl_ffi_node_create',
        ),
        nodeDestroy = library.lookupFunction<Void Function(Pointer<Void>),
            void Function(Pointer<Void>)>(
          'lwrcl_ffi_node_destroy',
        ),
        spinSome = library.lookupFunction<Void Function(Pointer<Void>),
            void Function(Pointer<Void>)>(
          'lwrcl_ffi_spin_some',
        ),
        bytesFree = library.lookupFunction<Void Function(Pointer<Uint8>),
            void Function(Pointer<Uint8>)>(
          'lwrcl_ffi_bytes_free',
        ),
        lastError = library.lookupFunction<Pointer<Utf8> Function(),
            Pointer<Utf8> Function()>(
          'lwrcl_ffi_last_error',
        );

  final DynamicLibrary _library;

  final void Function() lwrclInit;
  final void Function(int, Pointer<Pointer<Utf8>>) lwrclInitArgs;
  final void Function() lwrclShutdown;
  final int Function() lwrclOk;

  final Pointer<Void> Function(Pointer<Utf8>, Pointer<Utf8>) nodeCreate;
  final void Function(Pointer<Void>) nodeDestroy;
  final void Function(Pointer<Void>) spinSome;
  final void Function(Pointer<Uint8>) bytesFree;
  final Pointer<Utf8> Function() lastError;

  final Map<String, Pointer<Void> Function(Pointer<Void>, Pointer<Utf8>, int)>
      _publisherCreateCache = {};
  final Map<String, void Function(Pointer<Void>)> _publisherDestroyCache = {};
  final Map<String, int Function(Pointer<Void>, Pointer<Void>)>
      _publisherPublishCache = {};
  final Map<String, Pointer<Void> Function(Pointer<Void>, Pointer<Utf8>, int)>
      _subscriptionCreateCache = {};
  final Map<String, void Function(Pointer<Void>)> _subscriptionDestroyCache =
      {};
  final Map<String, int Function(Pointer<Void>)> _subscriptionHasMessageCache =
      {};
  final Map<String, int Function(Pointer<Void>, Pointer<Void>)>
      _subscriptionTakeCache = {};

  Pointer<Void> Function(Pointer<Void>, Pointer<Utf8>, int)
      lookupPublisherCreate(String symbol) {
    return _publisherCreateCache.putIfAbsent(
        symbol,
        () => _library.lookupFunction<
                Pointer<Void> Function(Pointer<Void>, Pointer<Utf8>, Uint16),
                Pointer<Void> Function(Pointer<Void>, Pointer<Utf8>, int)>(
              symbol,
            ));
  }

  void Function(Pointer<Void>) lookupPublisherDestroy(String symbol) {
    return _publisherDestroyCache.putIfAbsent(
        symbol,
        () => _library.lookupFunction<Void Function(Pointer<Void>),
                void Function(Pointer<Void>)>(
              symbol,
            ));
  }

  int Function(Pointer<Void>, Pointer<Void>) lookupPublisherPublish(
      String symbol) {
    return _publisherPublishCache.putIfAbsent(
        symbol,
        () => _library.lookupFunction<
                Int32 Function(Pointer<Void>, Pointer<Void>),
                int Function(Pointer<Void>, Pointer<Void>)>(
              symbol,
            ));
  }

  Pointer<Void> Function(Pointer<Void>, Pointer<Utf8>, int)
      lookupSubscriptionCreate(String symbol) {
    return _subscriptionCreateCache.putIfAbsent(
        symbol,
        () => _library.lookupFunction<
                Pointer<Void> Function(Pointer<Void>, Pointer<Utf8>, Uint16),
                Pointer<Void> Function(Pointer<Void>, Pointer<Utf8>, int)>(
              symbol,
            ));
  }

  void Function(Pointer<Void>) lookupSubscriptionDestroy(String symbol) {
    return _subscriptionDestroyCache.putIfAbsent(
        symbol,
        () => _library.lookupFunction<Void Function(Pointer<Void>),
                void Function(Pointer<Void>)>(
              symbol,
            ));
  }

  int Function(Pointer<Void>) lookupSubscriptionHasMessage(String symbol) {
    return _subscriptionHasMessageCache.putIfAbsent(
        symbol,
        () => _library.lookupFunction<Int32 Function(Pointer<Void>),
                int Function(Pointer<Void>)>(
              symbol,
            ));
  }

  int Function(Pointer<Void>, Pointer<Void>) lookupSubscriptionTake(
      String symbol) {
    return _subscriptionTakeCache.putIfAbsent(
        symbol,
        () => _library.lookupFunction<
                Int32 Function(Pointer<Void>, Pointer<Void>),
                int Function(Pointer<Void>, Pointer<Void>)>(
              symbol,
            ));
  }

  T lookupFunction<T extends Function, F extends Function>(String symbol) {
    return _library.lookupFunction<F, T>(symbol);
  }

  String? readLastError() {
    final ptr = lastError();
    if (ptr.address == 0) {
      return null;
    }
    return ptr.toDartString();
  }
}
