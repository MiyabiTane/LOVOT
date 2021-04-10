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

FROM_ADDRESS = 'tanemoto.jsk@gmail.com'
MY_PASSWORD = 'hogehoge'


def create_message(from_addr, to_addr, bcc_addrs, subject, body, img_path):
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


def send(from_addr, to_addrs, msg):
    #context = ssl.create_default_context()
    smtpobj = smtplib.SMTP_SSL('smtp.gmail.com', 465, timeout=10)
    smtpobj.login(FROM_ADDRESS, MY_PASSWORD)
    smtpobj.sendmail(from_addr, to_addrs, msg.as_string())
    smtpobj.close()


def send_mail_main(num, path):
    to_addr = 'tanemoto@jsk.imi.i.u-tokyo.ac.jp'
    BCC = ''
    subject = 'LOVOTの状態'
    img_path = path
    if num == 0:
        body = "LOVOTの充電がしばらく行われていません。ネストに戻れていない可能性があるので73B2付近の方はLOVOTをネストに戻してあげてください。"
    elif num == 1:
        body = "LOVOTがネスト付近にいますが充電できていません。ネストとLOVOTの充電端子が接続されているか確かめて下さい。"
    elif num == 2:
        body = "おはようございます。今日のLOVOTの様子です。"

    msg = create_message(FROM_ADDRESS, to_addr, BCC, subject, body, img_path)
    send(FROM_ADDRESS, to_addr, msg)


def send_mail_debug(path):
    to_addr = 'tanemoto@jsk.imi.i.u-tokyo.ac.jp'
    BCC = ''
    subject = 'デバッグ用_LOVOTの充電開始'
    img_path = path
    body = "LOVOTが充電中です。"

    msg = create_message(FROM_ADDRESS, to_addr, BCC, subject, body, img_path)
    send(FROM_ADDRESS, to_addr, msg)


def send_mail_debug_staus(info, path):
    to_addr = 'tanemoto@jsk.imi.i.u-tokyo.ac.jp'
    BCC = ''
    subject = 'デバッグ用_LOVOTの状態'
    img_path = path

    check_hour, check_minute, status = info
    status_memo = "充電中です。" if status == 1 else "活動中です。"
    body = "LOVOTチェッカーのステータスをお知らせします。" + str(check_hour) + "時" + str(check_minute) + "分から" + status_memo

    msg = create_message(FROM_ADDRESS, to_addr, BCC, subject, body, img_path)
    send(FROM_ADDRESS, to_addr, msg)
