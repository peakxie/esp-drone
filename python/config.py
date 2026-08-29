#!/usr/bin/env python3
# 公共连接配置 + 建链辅助：所有测试脚本共用一个 URI 和同一套带超时的连接逻辑，改这里一处即可。

import threading

URI = "udp://192.168.43.42:2390"  # 换成你能连上的地址

CONNECT_TIMEOUT_S = 10.0  # 建链超时：飞机没应答时 cflib 会无提示地一直重试，超过这个时间主动放弃并打印原因


def connect_with_timeout(cf, uri=URI, timeout_s=CONNECT_TIMEOUT_S):
    """带超时的建链。cflib 在链路完全没有应答时会无提示地一直重试，表现为"卡死无反应"
    （比如飞机没通电、PC 没连上同一个 Wi-Fi、IP/端口不对，或者上一次遗留的旧进程还占着地址）。
    这里主动等一个有限时间，超时或失败都打印原因并 close_link()，不再让人干等猜测。"""
    result = {"failed_msg": None}
    connected_evt = threading.Event()

    def on_connected(_uri):
        connected_evt.set()

    def on_connection_failed(_uri, msg):
        result["failed_msg"] = msg
        connected_evt.set()

    cf.connected.add_callback(on_connected)
    cf.connection_failed.add_callback(on_connection_failed)

    print(f"正在连接 {uri} （最长等待 {timeout_s:.0f}s）...", flush=True)
    cf.open_link(uri)

    ok = connected_evt.wait(timeout=timeout_s)
    cf.connected.remove_callback(on_connected)
    cf.connection_failed.remove_callback(on_connection_failed)

    if not ok:
        print(
            f"连接超时（{timeout_s:.0f}s 内没有任何应答）：飞机可能没通电、没连上同一个 Wi-Fi、"
            "IP/端口不对，或者上一次遗留的旧进程还占着地址（去任务管理器确认没有残留的 python.exe）。"
            "请检查后重试。",
            flush=True,
        )
        cf.close_link()
        return False

    if result["failed_msg"] is not None:
        print(f"连接失败：{result['failed_msg']}", flush=True)
        return False

    print("已连接。", flush=True)
    return True
