# -*- coding: utf-8 -*-
import smtplib
from email.mime.text import MIMEText
from email.utils import formatdate
import ssl

FROM_ADDRESS = 'tanemoto.jsk@gmail.com'
MY_PASSWORD = 'hogehoge'


def create_message(from_addr, to_addr, bcc_addrs, subject, body):
    msg = MIMEText(body)
    msg['Subject'] = subject
    msg['From'] = from_addr
    msg['To'] = to_addr
    msg['Bcc'] = bcc_addrs
    msg['Date'] = formatdate()
    return msg


def send(from_addr, to_addrs, msg):
    #context = ssl.create_default_context()
    smtpobj = smtplib.SMTP_SSL('smtp.gmail.com', 465, timeout=10)
    smtpobj.login(FROM_ADDRESS, MY_PASSWORD)
    smtpobj.sendmail(from_addr, to_addrs, msg.as_string())
    smtpobj.close()


def send_mail_main(num):
    to_addr = 'tanemoto@jsk.imi.i.u-tokyo.ac.jp'
    BCC = ''
    subject = 'LOVOTの状態'
    if num == 0:
        body = "LOVOTの充電がしばらく行われていません。ネストに戻れていない可能性があるので73B2付近の方はLOVOTをネストに戻してあげてください。"
    elif num == 1:
        body = "LOVOTがネスト付近にいますが充電できていません。ネストとLOVOTの充電端子が接続されているか確かめて下さい。"
    elif num == 2:
        body = "LOVOTが充電中です。"

    msg = create_message(FROM_ADDRESS, to_addr, BCC, subject, body)
    send(FROM_ADDRESS, to_addr, msg)