import 'dart:async';

import 'package:lwrcl_dart/lwrcl_dart.dart';

void main() {
  Lwrcl.init();

  final node = Node.create('dart_node');
  final pub = node.createPublisher<StdMsgsString>(
      StdMsgsString.type, 'chatter');
  final sub = node.createSubscription<StdMsgsString>(
      StdMsgsString.type, () => StdMsgsString(), 'chatter');

  final timer = Timer.periodic(const Duration(milliseconds: 500), (_) {
    final msg = StdMsgsString();
    msg.data = 'hello from dart';
    pub.publish(msg);
    msg.dispose();

    node.spinSome();
    final received = sub.take();
    if (received != null) {
      // ignore: avoid_print
      print('received: ${received.data}');
      received.dispose();
    }
  });

  // Stop after 10 seconds.
  Timer(const Duration(seconds: 10), () {
    timer.cancel();
    sub.dispose();
    pub.dispose();
    node.dispose();
    Lwrcl.shutdown();
  });
}
