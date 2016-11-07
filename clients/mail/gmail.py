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
#    Created by Philip Joseph <jphilip09@gmail.com>
#
#    Using sample code from
#    https://developers.google.com/gmail/api/quickstart/python

from __future__ import print_function
import base64
from datetime import datetime
from email.mime.text import MIMEText
import httplib2
import logging
import os
import sys

from apiclient import discovery
from oauth2client import client
from oauth2client.file import Storage
from oauth2client import tools

try:
    import argparse
    flags = argparse.ArgumentParser(parents=[tools.argparser]).parse_args()
except ImportError:
    flags = None

log = logging.getLogger('jr-home-automation')


class GmailIntf(object):

    # If modifying these scopes, delete your previously saved credentials
    # at ~/.credentials/gmail-python-quickstart.json
    SCOPES = ['https://www.googleapis.com/auth/gmail.send',
              'https://www.googleapis.com/auth/gmail.readonly']

    def __init__(self, config):

        self._app_name = config.get('application-name')
        self._config_dir = config.get('config-dir')
        self._secrets_file = config.get('secrets-file')
        if self._secrets_file is None:
            raise Exception("Secrets file for gmail not provided!!")

        self._from = config.get('from-id')
        if self._from is None:
            raise Exception("From email ID not provided!!")

        self._to = ''
        to_ids = config.get('to-id')
        if not to_ids:
            raise Exception("To email IDs not provided!!")

        if isinstance(to_ids, list):
            for id_ in to_ids:
                if len(self._to):
                    self._to += ';' + id_
                else:
                    self._to = id_
        else:
            self._to = to_ids

        self._priority = self._to
        priority_ids = config.get('priority-id')
        if not priority_ids:
            log.warning("Priority email IDs not provided!")

        if isinstance(priority_ids, list):
            for id_ in priority_ids:
                if len(self._priority):
                    self._priority += ';' + id_
                else:
                    self._priority = id_
        else:
            if len(self._priority):
                self._priority += ';' + priority_ids
            else:
                self._priority = priority_ids

        log.debug("From: {}, To: {}, Priority: {}".
                  format(self._from, self._to, self._priority))

    def get_credentials(self):
        """Gets valid user credentials from storage.

        If nothing has been stored, or if the stored credentials are invalid,
        the OAuth2 flow is completed to obtain the new credentials.

        Returns:
        Credentials, the obtained credential.
        """
        credential_dir = self._config_dir
        if not os.path.exists(credential_dir):
            os.makedirs(credential_dir)
        credential_path = os.path.join(credential_dir,
                                       '.jr-home-automation-gmail.json')

        store = Storage(credential_path)
        credentials = store.get()
        if not credentials or credentials.invalid:
            if self._secrets_file.startswith('/'):
                secret_file = self._secrets_file
            else:
                secret_file = os.path.join(self._config_dir,
                                           self._secrets_file)

                flow = client.flow_from_clientsecrets(secret_file,
                                                      self.SCOPES)
                flow.user_agent = self._app_name
            if flags:
                credentials = tools.run_flow(flow, store, flags)
            else:  # Needed only for compatibility with Python 2.6
                credentials = tools.run(flow, store)
            log.info('Storing credentials to ' + credential_path)
        return credentials

    def create_message(self, sender, to, subject, message_text):
        """Create a message for an email.

        Args:
          sender: Email address of the sender.
          to: Email address of the receiver.
          subject: The subject of the email message.
          message_text: The text of the email message.

        Returns:
          An object containing a base64url encoded email object.
        """

        message = MIMEText(message_text)
        message['to'] = to
        message['from'] = sender
        message['subject'] = subject
        return {'raw':
                base64.urlsafe_b64encode(message.as_string().
                                         encode()).decode()}

    def send_message(self, service, user_id, message):
        """Send an email message.

        Args:
          service: Authorized Gmail API service instance.
          user_id: User's email address. The special value "me"
          can be used to indicate the authenticated user.
          message: Message to be sent.

        Returns:
         Sent Message.
        """

        try:
            message = (service.users().messages().send(userId=user_id,
                                                       body=message)
                       .execute())
            log.info('Message Id: %s' % message['id'])
            return message
        except Exception as error:
            log.exception('An error occurred: %s' % error)

    def send(self, subject, message_text, priority=False):
        credentials = self.get_credentials()
        http = credentials.authorize(httplib2.Http())
        service = discovery.build('gmail', 'v1', http=http)

        if priority:
            ids = self._priority
        else:
            ids = self._to

        msg = self.create_message(self._from,
                                  ids,
                                  subject,
                                  message_text)
        log.debug("Message: {}".format(msg))
        self.send_message(service, 'me', msg)

    def get_labels(self):
        credentials = self.get_credentials()
        http = credentials.authorize(httplib2.Http())
        service = discovery.build('gmail', 'v1', http=http)

        results = service.users().labels().list(userId='me').execute()
        labels = results.get('labels', [])

        log.debug("Labels: {}".format(labels))
        return labels


def main():
    """Test the Gmail interface

    Creates a Gmail API service object, outputs a list of label names
    of the user's Gmail account and send test mails
    """

    fmt = logging.Formatter(
        '%(asctime)-23s %(levelname)-5s  (%(name)s@%(process)d:'
        '%(filename)s:%(lineno)d) - %(message)s')
    stderr_handler = logging.StreamHandler(stream=sys.stderr)
    stderr_handler.setFormatter(fmt)
    log.setLevel(logging.DEBUG)
    log.addHandler(stderr_handler)

    config = {
        'from-id': 'jrhomeauto@gmail.com',
        'to-id': ['jphilip09@gmail.com'],
        'priority-id': ['joerekha@gmail.com'],
        'config-dir': os.path.join(os.path.expanduser('~'),
                                   '.jrhomeauto'),
        'secrets-file': 'client_secret.json',
        'application-name': 'JR Home Automation',
    }

    intf = GmailIntf(config)

    labels = intf.get_labels()
    if not labels:
        print('No labels found.')
    else:
        print('Labels:')
        for label in labels:
            print(label['name'])

    subject = 'Test ' + datetime.isoformat(datetime.now())
    intf.send(subject, '')
    intf.send('Priority: '+subject, '', True)

if __name__ == '__main__':
    main()
