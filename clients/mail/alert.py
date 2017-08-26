#!/usr/bin/env python3

#    Licensed under the Apache License, Version 2.0 (the "License"); you may
#    not use this file except in compliance with the License. You may obtain
#    a copy of the License at
#
#         http://www.apache.org/licenses/LICENSE-2.0
#
#    Unless required by applicable law or agreed to in writing, software
#    distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
#    WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
#    License for the specific language governing permissions and limitations
#    under the License.
#
#
# Add a client to subscribe to MQTT and send mail
#
# Created by Philip Joseph <jphilip09@gmail.com>
#
# References:
#    hbmqtt example: https://github.com/beerfactory/hbmqtt/blob/master/samples/client_subscribe.py
#
# NOTE: Not yet complete!!


import argparse
import asyncio
from datetime import datetime
import logging
import math
import os
import time
import yaml

from hbmqtt.client import ClientException
from hbmqtt.client import MQTTClient

from .gmail import GmailIntf

log = logging.getLogger('jr-home-automation')


def calculate_volume(height, radius):
    volume = math.pi * math.sqrt(radius) * height
    return volume/1000


class OverheadTank(object):
    def __init__(height, radius, client):
        self._max_ht = height
        self._rad = radius
        self._client = client

        self._last_ht = 0
        self._last_vol = 0
        self._max_vol = calculate_volume(self._max_ht, self._rad)
        log.debug("Overhead tank maximum volume: {}".format(self._max_vol))

        self._priority_th = height * 20 / 100  # Alarm at or below 20 %
        log.debug("Overhead tank priority threshold: {} ({})".
                  format(self._priority_th, height))

    @property
    def height(self):
        return self._last_ht

    @property
    def volume(self):
        return self._last_volume

    def update(self, height):
        self._last_ht = height
        self._last_vol = calculate_volume(height, self._rad)
        p = height * 100 / self._max_ht  # Percentage
        priority = False
        if height <= self._priority_th:
            priority = True

        header = "Overhead tank water level: {} ({}%)".format(height, p)
        body = "Overhead Tank\n\tCurrent height of water: {} ({}%)" + \
               "\n\tCurrent volume of water: {}". \
               format(height, p, self._last_vol)
        yield from client.update(header, body, priority)


class Client(object):

    def __init__(self, loop, config):
        self._loop = loop
        self._gmail = GmailIntf(config)

    @asyncio.coroutine
    def update(self, header, body, priority=False):
        subject = '[JR-HA] ' + header
        self._gmail.send(subject, body, priority)


class Notify(object):
    APPLICATION_NAME = 'JR Home Automation'
    CONFIG_DIR = '.jrhomeauto'
    CONFIG_FILE = 'jrhomeauto.conf'

    config = {}
    home_dir = os.path.expanduser('~')

    @classmethod
    def get_config(cls, file=None):
        if config:
            return config

        # Read the config file
        config = {
            'application-name': cls.APPLICATION_NAME,
            'mqtt-topic': [('$SYS/broker/uptime', QOS_1)],
            'mqtt-url': 'mqtt://test.mosquitto.org/',
            'from-id': None,
            'to-id': [],
            'priority-id': [],
        }

        if not file:
            file = os.path.join(home_dir, cls.CONFIG_DIR, cls.CONFIG_FILE)
        if not os.path.isfile(file):
            raise Exception("Unable to read config file: {}".format(file))

        if not os.path.isdir(cls.CONFIG_DIR):
            os.mkdir(cls.CONFIG_DIR, mode=0o700)

        config['config-dir'] = cls.CONFIG_DIR

        with open(file, 'r') as f:
            config.update(yaml.load(f))

        config['config-file'] = file

        return settings

    def __init__(self, args, loop):
        self._loop = loop
        Notify.get_config()
        client = Client(loop, config)

    @asyncio.coroutine
    def loop(self):
        C = MQTTClient()
        yield from C.connect(config.get('mqtt-url'))
        log.info("Connected to MQTT: {}".format(config.get('mqtt-url')))

        yield from C.subscribe(
            config.get('mqtt-topic'),
        )
        log.info(" MQTT subscribed to {}".format(config.get('mqtt-topic')))

        try:
            while True:
                message = yield from C.deliver_message()
                packet = message.publish_packet
                topic = packet.variable_header.topic_name
                payload = packet.payload.data
                log.debug("Got message: %s => %s" % (topic, str(payload)))

        except ClientException as ce:
            log.execption(ce)
        finally:
            yield from C.disconnect()


def main(args):
    loop = asyncio.get_event_loop()
    notify = Notify(args, loop)
    loop.run_until_complete(notify.loop())


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Alert for Home Automation')
    parser.add_argument("-m", "--mqtt-server", default='localhost',
                        help="MQTT server")
    parser.add_argument("-p", "--port", default=1883,
                        help="Port number, default 1883")
    parser.add_argument("-d", "--debug", action='store_true')
    args = parser.parse_args()

    fmt = logging.Formatter(
        '%(asctime)-23s %(levelname)-5s  (%(name)s@%(process)d:'
        '%(filename)s:%(lineno)d) - %(message)s')
    stderr_handler = logging.StreamHandler(stream=sys.stderr)
    stderr_handler.setFormatter(fmt)
    level = logging.INFO
    if args.debug:
        level = logging.DEBUG
    filename = os.path.basename(sys.argv[0]) + '.log'

    logging.basicConfig(filename=filename, level=level)
    log.addHandler(stderr_handler)

    try:
        main(args)

    except Exception as e:
        log.execption(e)
