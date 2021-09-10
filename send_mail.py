# -*- coding: utf-8 -*-
import os
import glob
import smtplib
from email import encoders
from email.mime.text import MIMEText
from email.mime.base import MIMEBase
from email.mime.multipart import MIMEMultipart
from email.utils import formatdate
import ssl


class SEND_MAIL:
    def __init__(self, from_addr, password, to_addr_p, to_addr_o):
        self.FROM_ADDRESS = from_addr
        self.MY_PASSWORD = password
        self.TO_ADDR_P = to_addr_p
        self.TO_ADDR_O = to_addr_o


    def create_message(self, to_addr, subject, body, img_path):
        from_addr=self.FROM_ADDRESS
        bcc_addrs = ''
        msg = MIMEMultipart()
        msg['Subject'] = subject
        msg['From'] = from_addr
        msg['To'] = to_addr
        msg['Bcc'] = bcc_addrs
        msg['Date'] = formatdate()

        # 本文
        text_body = MIMEText(body)
        msg.attach(text_body)

        # 画像添付
        attach_file = {'name': 'nest_view.jpg', 'path': img_path}
        attachment = MIMEBase('image', 'png')
        file_ = open(attach_file['path'], 'rb+')
        attachment.set_payload(file_.read())
        file_.close()
        encoders.encode_base64(attachment)
        attachment.add_header("Content-Disposition", "attachment", filename=attach_file['name'])
        msg.attach(attachment)

        return msg


    def send(self, to_addr, msg):
        from_addr=self.FROM_ADDRESS
        #context = ssl.create_default_context()
        smtpobj = smtplib.SMTP_SSL('smtp.gmail.com', 465, timeout=10)
        smtpobj.login(from_addr, self.MY_PASSWORD)
        smtpobj.sendmail(from_addr, to_addr, msg.as_string())
        smtpobj.close()


    def send_mail_main(self, num, path):
        subject = 'LOVOTの状態'
        img_path = path
        if num == 0:
            body = "LOVOTの充電がしばらく行われていません。ネストに戻れていない可能性があるので73B2付近の方はLOVOTをネストに戻してあげてください。"
        elif num == 1:
            body = "LOVOTがネスト付近にいますが充電できていません。ネストとLOVOTの充電端子が接続されているか確かめて下さい。"
        elif num == 2:
            body = "おはようございます。今日のLOVOTの様子です。"

        msg = self.create_message(to_addr=self.TO_ADDR_O, subject=subject, body=body, img_path=img_path)
        self.send(to_addr=self.TO_ADDR_O, msg=msg)


    def send_mail_debug(self, path):
        subject = 'デバッグ用_LOVOTの充電開始'
        img_path = path
        body = "LOVOTが充電中です。"

        msg = self.create_message(to_addr=self.TO_ADDR_P, subject=subject, body=body, img_path=img_path)
        send(to_addr=self.TO_ADDR_P, msg=msg)


    def send_mail_debug_staus(self, info, path):
        subject = 'デバッグ用'
        img_path = path

        check_hour, check_minute, status = info
        status_memo = "充電中です。" if status == 1 else "活動中です。"
        body = "LOVOTチェッカーのステータスをお知らせします。" + str(check_hour) + "時" + str(check_minute) + "分から" + status_memo

        msg = self.create_message(to_addr=self.TO_ADDR_P, subject=subject, body=body, img_path=img_path)
        self.send(to_addr=self.TO_ADDR_P, msg=msg)

