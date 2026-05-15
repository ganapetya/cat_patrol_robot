#!/usr/bin/env python3
"""Subscribe to JSON mail requests (from patrol_node) and send via SMTP.

Configure with environment variables (never commit secrets):
  CAT_PATROL_SMTP_HOST   e.g. smtp.gmail.com
  CAT_PATROL_SMTP_PORT   default 587
  CAT_PATROL_SMTP_USER
  CAT_PATROL_SMTP_PASSWORD
  CAT_PATROL_SMTP_FROM   optional, defaults to USER

Message format on std_msgs/String (JSON):
  {"subject":"...", "to":"...", "paths":["/abs/a.jpg", ...]}
"""
import json
import mimetypes
import os
import smtplib
from email.message import EmailMessage

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MailNode(Node):
    def __init__(self):
        super().__init__('cat_patrol_mail')
        self.declare_parameter('mail_request_topic', '/cat_patrol/mail_request')
        topic = self.get_parameter('mail_request_topic').get_parameter_value().string_value
        self.sub = self.create_subscription(String, topic, self._cb, 10)

    def _cb(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error('Bad JSON: %s' % e)
            return
        subject = data.get('subject', 'Cat patrol')
        to_addr = data.get('to', '')
        paths = data.get('paths', [])
        if not to_addr:
            self.get_logger().warning('No "to" address in mail request; skipping')
            return

        host = os.environ.get('CAT_PATROL_SMTP_HOST', '')
        user = os.environ.get('CAT_PATROL_SMTP_USER', '')
        password = os.environ.get('CAT_PATROL_SMTP_PASSWORD', '')
        from_addr = os.environ.get('CAT_PATROL_SMTP_FROM', user)
        port = int(os.environ.get('CAT_PATROL_SMTP_PORT', '587'))

        if not host or not user or not password:
            self.get_logger().warning(
                'SMTP not configured (set CAT_PATROL_SMTP_* env); mail not sent. '
                'Request had %d paths.' % len(paths)
            )
            return

        m = EmailMessage()
        m['Subject'] = subject
        m['From'] = from_addr
        m['To'] = to_addr
        m.set_content('Cat patrol snapshot batch.\n')

        for p in paths:
            if not os.path.isfile(p):
                self.get_logger().warning('Missing attachment: %s' % p)
                continue
            ctype, _ = mimetypes.guess_type(p)
            if ctype is None:
                ctype = 'application/octet-stream'
            maintype, subtype = ctype.split('/', 1)
            with open(p, 'rb') as f:
                m.add_attachment(
                    f.read(), maintype=maintype, subtype=subtype, filename=os.path.basename(p)
                )

        try:
            with smtplib.SMTP(host, port) as s:
                s.starttls()
                s.login(user, password)
                s.send_message(m)
            self.get_logger().info('Mail sent to %s (%d files).' % (to_addr, len(paths)))
        except Exception as e:
            self.get_logger().error('SMTP failed: %s' % e)


def main():
    rclpy.init()
    n = MailNode()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
