#!/usr/bin/env python3

from scapy.all import *
import argparse
import sys

def main():
    # 配置命令行参数
    parser = argparse.ArgumentParser(description='Convert raw binary packet to PCAP')
    parser.add_argument('-i', '--input', required=True, help='Input raw binary file')
    parser.add_argument('-o', '--output', default='output.pcap', help='Output PCAP file')
    args = parser.parse_args()

    try:
        # 读取二进制文件
        with open(args.input, 'rb') as f:
            raw_bytes = f.read()

        # 自动解析协议栈（推荐方式）
        packet = Ether(raw_bytes)
        
        # 保存PCAP文件
        wrpcap(args.output, [packet])
        print(f"[+] Successfully generated: {args.output}")
        print(f"    Packet length: {len(raw_bytes)} bytes")
        print(f"    Link layer: {packet.name}")
        print(f"    Network layer: {packet[1].name if len(packet) > 1 else 'Unknown'}")

    except FileNotFoundError:
        print(f"[!] Error: Input file '{args.input}' not found", file=sys.stderr)
    except Exception as e:
        print(f"[!] Processing error: {str(e)}", file=sys.stderr)

if __name__ == "__main__":
    main()
