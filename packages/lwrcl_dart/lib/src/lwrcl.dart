import 'dart:ffi';

import 'package:ffi/ffi.dart';

import 'bindings.dart';

final LwrclBindings bindings = LwrclBindings(openLwrclLibrary());

class LwrclException implements Exception {
  LwrclException(this.message);
  final String message;

  @override
  String toString() => 'LwrclException: $message';
}

void throwIfLastError(String fallbackMessage) {
  final message = bindings.readLastError();
  if (message != null && message.isNotEmpty) {
    throw LwrclException(message);
  }
}

Never throwWithFallback(String fallbackMessage) {
  final message = bindings.readLastError();
  if (message != null && message.isNotEmpty) {
    throw LwrclException(message);
  }
  throw LwrclException(fallbackMessage);
}

class MessageType {
  const MessageType(this.package, this.name);

  final String package;
  final String name;

  String get suffix => '${package}__msg__${name}';
}

class MessageBase {
  MessageBase(this._handle, this._destroy) : _ownsHandle = true;

  MessageBase.fromHandle(this._handle, this._destroy) : _ownsHandle = true;

  final Pointer<Void> _handle;
  final void Function(Pointer<Void>) _destroy;
  bool _ownsHandle;

  Pointer<Void> get handle => _handle;

  void dispose() {
    if (_ownsHandle) {
      _destroy(_handle);
      final error = bindings.readLastError();
      if (error != null && error.isNotEmpty) {
        throw LwrclException(error);
      }
      _ownsHandle = false;
    }
  }
}

class Lwrcl {
  static void init({List<String> args = const []}) {
    if (args.isEmpty) {
      bindings.lwrclInit();
      final error = bindings.readLastError();
      if (error != null && error.isNotEmpty) {
        throw LwrclException(error);
      }
      return;
    }

    final argv = calloc<Pointer<Utf8>>(args.length);
    try {
      for (var i = 0; i < args.length; i++) {
        argv[i] = args[i].toNativeUtf8();
      }
      bindings.lwrclInitArgs(args.length, argv);
      final error = bindings.readLastError();
      if (error != null && error.isNotEmpty) {
        throw LwrclException(error);
      }
    } finally {
      for (var i = 0; i < args.length; i++) {
        malloc.free(argv[i]);
      }
      calloc.free(argv);
    }
  }

  static void shutdown() {
    bindings.lwrclShutdown();
    final error = bindings.readLastError();
    if (error != null && error.isNotEmpty) {
      throw LwrclException(error);
    }
  }

  static bool ok() {
    final result = bindings.lwrclOk();
    final error = bindings.readLastError();
    if (error != null && error.isNotEmpty) {
      throw LwrclException(error);
    }
    return result == 1;
  }
}

class Node {
  Node._(this._handle);

  final Pointer<Void> _handle;

  static Node create(String name, {String? namespace}) {
    final namePtr = name.toNativeUtf8();
    final nsPtr = (namespace == null || namespace.isEmpty)
        ? Pointer<Utf8>.fromAddress(0)
        : namespace.toNativeUtf8();
    try {
      final handle = bindings.nodeCreate(namePtr, nsPtr);
      if (handle.address == 0) {
        throwWithFallback('Node.create failed');
      }
      return Node._(handle);
    } finally {
      malloc.free(namePtr);
      if (nsPtr.address != 0) {
        malloc.free(nsPtr);
      }
    }
  }

  void spinSome() {
    bindings.spinSome(_handle);
    final error = bindings.readLastError();
    if (error != null && error.isNotEmpty) {
      throw LwrclException(error);
    }
  }

  Publisher<T> createPublisher<T extends MessageBase>(
      MessageType type, String topic,
      {int depth = 10}) {
    final topicPtr = topic.toNativeUtf8();
    try {
      final createSymbol = 'lwrcl_ffi_publisher_create__${type.suffix}';
      final publishSymbol = 'lwrcl_ffi_publisher_publish__${type.suffix}';
      final destroySymbol = 'lwrcl_ffi_publisher_destroy__${type.suffix}';

      final create = bindings.lookupPublisherCreate(createSymbol);
      final handle = create(_handle, topicPtr, depth);
      if (handle.address == 0) {
        throwWithFallback('createPublisher failed');
      }

      return Publisher._(
        handle,
        bindings.lookupPublisherPublish(publishSymbol),
        bindings.lookupPublisherDestroy(destroySymbol),
      );
    } finally {
      malloc.free(topicPtr);
    }
  }

  Subscription<T> createSubscription<T extends MessageBase>(
      MessageType type, T Function() factory, String topic,
      {int depth = 10}) {
    final topicPtr = topic.toNativeUtf8();
    try {
      final createSymbol = 'lwrcl_ffi_subscription_create__${type.suffix}';
      final destroySymbol = 'lwrcl_ffi_subscription_destroy__${type.suffix}';
      final hasSymbol = 'lwrcl_ffi_subscription_has_message__${type.suffix}';
      final takeSymbol = 'lwrcl_ffi_subscription_take__${type.suffix}';

      final create = bindings.lookupSubscriptionCreate(createSymbol);
      final handle = create(_handle, topicPtr, depth);
      if (handle.address == 0) {
        throwWithFallback('createSubscription failed');
      }

      return Subscription._(
        handle,
        factory,
        bindings.lookupSubscriptionHasMessage(hasSymbol),
        bindings.lookupSubscriptionTake(takeSymbol),
        bindings.lookupSubscriptionDestroy(destroySymbol),
      );
    } finally {
      malloc.free(topicPtr);
    }
  }

  void dispose() {
    bindings.nodeDestroy(_handle);
    final error = bindings.readLastError();
    if (error != null && error.isNotEmpty) {
      throw LwrclException(error);
    }
  }
}

class Publisher<T extends MessageBase> {
  Publisher._(this._handle, this._publish, this._destroy);

  final Pointer<Void> _handle;
  final int Function(Pointer<Void>, Pointer<Void>) _publish;
  final void Function(Pointer<Void>) _destroy;

  void publish(T msg) {
    final result = _publish(_handle, msg.handle);
    if (result != 0) {
      throwWithFallback('publish failed');
    }
  }

  void dispose() {
    _destroy(_handle);
    final error = bindings.readLastError();
    if (error != null && error.isNotEmpty) {
      throw LwrclException(error);
    }
  }
}

class Subscription<T extends MessageBase> {
  Subscription._(
      this._handle, this._factory, this._hasMessage, this._take, this._destroy);

  final Pointer<Void> _handle;
  final T Function() _factory;
  final int Function(Pointer<Void>) _hasMessage;
  final int Function(Pointer<Void>, Pointer<Void>) _take;
  final void Function(Pointer<Void>) _destroy;

  bool hasMessage() {
    final result = _hasMessage(_handle);
    final error = bindings.readLastError();
    if (error != null && error.isNotEmpty) {
      throw LwrclException(error);
    }
    return result == 1;
  }

  T? take() {
    final msg = _factory();
    final result = _take(_handle, msg.handle);
    if (result == 0) {
      msg.dispose();
      return null;
    }
    if (result < 0) {
      msg.dispose();
      throwWithFallback('take failed');
    }
    return msg;
  }

  void dispose() {
    _destroy(_handle);
    final error = bindings.readLastError();
    if (error != null && error.isNotEmpty) {
      throw LwrclException(error);
    }
  }
}
