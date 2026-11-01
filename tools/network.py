#!/usr/bin/env python3
"""Write a per-device ROS DDS profile. Never stores Wi-Fi passwords."""
import argparse
import ipaddress
from pathlib import Path
import re
import shlex
from xml.sax.saxutils import escape


def configure(interface, peer, domain, output=None):
    if not re.fullmatch(r'[A-Za-z0-9_.:-]+',interface):raise ValueError('网卡名称无效')
    ipaddress.ip_address(peer)
    if not 0<=domain<=232:raise ValueError('ROS_DOMAIN_ID 使用 0～232')
    directory=output or Path.home()/'.config/wheeltec_ros';directory.mkdir(parents=True,exist_ok=True)
    xml=f'''<CycloneDDS xmlns="https://cdds.io/config"><Domain id="any"><General><Interfaces><NetworkInterface name="{escape(interface)}"/></Interfaces><AllowMulticast>spdp</AllowMulticast><MaxMessageSize>1400B</MaxMessageSize><FragmentSize>1200B</FragmentSize></General><Discovery><Peers><Peer address="{peer}"/></Peers></Discovery></Domain></CycloneDDS>'''
    (directory/'network.xml').write_text(xml,encoding='utf-8')
    text='unset FASTRTPS_DEFAULT_PROFILES_FILE RMW_FASTRTPS_PUBLICATION_MODE RMW_FASTRTPS_USE_QOS_FROM_XML\n'
    text+=f'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp\nexport ROS_DOMAIN_ID={domain}\nexport ROS_LOCALHOST_ONLY=0\n'
    text+='export CYCLONEDDS_URI='+shlex.quote('file://'+str((directory/'network.xml').resolve()))+'\n'
    (directory/'network.sh').write_text(text,encoding='utf-8');print(directory/'network.sh')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--interface',required=True);p.add_argument('--peer',required=True);p.add_argument('--domain',type=int,default=20)
    a=p.parse_args();configure(a.interface,a.peer,a.domain)
