"""Windows operator console. SSH only; flight starts only on the Start button."""
import json
from pathlib import Path
import threading
import tkinter as tk
from tkinter import ttk, messagebox
import webbrowser

import paramiko


def remote_command(action, route=1):
    if type(route) is not int or route not in (1, 2, 3, 4):
        raise ValueError("Route must be 1..4")
    ctl = 'bash "$HOME/simple_flight_ws/competitionctl.sh"'
    if action in ("check", "vision", "status", "stop", "stop-vision"):
        return f"{ctl} {action} {route}"
    if action == "run":
        # Detached from SSH; disconnecting the laptop cannot kill the controller.
        return (f'nohup {ctl} run {route} > "$HOME/simple_flight_ws/log/console_run.log" '
                '2>&1 < /dev/null & mission_pid=$!; sleep .5; '
                'if kill -0 "$mission_pid" 2>/dev/null; then printf "started_pid=%s\\n" "$mission_pid"; '
                'else cat "$HOME/simple_flight_ws/log/console_run.log"; exit 2; fi')
    if action in ("1-open", "1-close", "2-open", "2-close"):
        door, position = action.split("-")
        return ('source /opt/ros/humble/setup.bash\n'
                'source "$HOME/simple_flight_ws/install/setup.bash"\n'
                'export ROS_DOMAIN_ID=0\nunset ROS_LOCALHOST_ONLY\n'
                f'ros2 run simple_flight_payload doorctl {door} {position} '
                '--config "$HOME/simple_flight_ws/config/doors.json" --ground-test --execute')
    raise ValueError("Unsupported console action")


class Console:
    def __init__(self, root):
        self.root = root
        root.title("无人机比赛控制台")
        root.geometry("820x620")
        self.client = None
        self.prepared = None
        self.busy = False
        self.host = tk.StringVar(value="10.42.0.1")
        self.user = tk.StringVar(value="jetson")
        self.password = tk.StringVar()
        self.route = tk.IntVar(value=1)
        self.route.trace_add("write", lambda *_: self.invalidate())
        frame = ttk.Frame(root, padding=16)
        frame.pack(fill="both", expand=True)
        ttk.Label(frame, text="无人机比赛 · 现场控制", font=("Microsoft YaHei UI", 16)).pack(anchor="w")
        ttk.Label(frame, text="启动前：定位就绪、Position档、SWA低位、油门低位；镜头朝下，180°关门 / 0°开门。", wraplength=780).pack(anchor="w", pady=8)
        connection = ttk.Frame(frame)
        connection.pack(fill="x", pady=6)
        for label, variable, width in (("地址", self.host, 16), ("账号", self.user, 10), ("密码", self.password, 14)):
            ttk.Label(connection, text=label).pack(side="left", padx=(0, 4))
            ttk.Entry(connection, textvariable=variable, width=width, show="*" if label == "密码" else "").pack(side="left", padx=(0, 10))
        ttk.Button(connection, text="连接", command=self.connect).pack(side="left")
        routes = ttk.Frame(frame)
        routes.pack(fill="x", pady=8)
        ttk.Label(routes, text="障碍布局 / 路线：").pack(side="left")
        for number, layout in enumerate(("(1,1)", "(1,2)", "(2,1)", "(2,2)"), 1):
            ttk.Radiobutton(routes, text=f"{number} {layout}", variable=self.route, value=number).pack(side="left", padx=8)
        buttons = ttk.Frame(frame)
        buttons.pack(fill="x", pady=8)
        ttk.Button(buttons, text="1. 画面与录像", command=lambda: self.action("vision")).pack(side="left", padx=(0, 8))
        ttk.Button(buttons, text="2. 起飞前检查", command=lambda: self.action("check")).pack(side="left", padx=(0, 8))
        self.start = ttk.Button(buttons, text="3. 启动飞行", command=lambda: self.action("run"), state="disabled")
        self.start.pack(side="left", padx=(0, 8))
        ttk.Button(buttons, text="停止任务 / 遥控接管", command=lambda: self.action("stop")).pack(side="left")
        doors = ttk.Frame(frame)
        doors.pack(fill="x", pady=8)
        ttk.Label(doors, text="拆桨调门（默认180°；开=0°后回180°）：").pack(side="left")
        for action, label in (("1-close", "1关180°"), ("1-open", "1开0°"), ("2-close", "2关180°"), ("2-open", "2开0°")):
            ttk.Button(doors, text=label, command=lambda a=action: self.action(a)).pack(side="left", padx=4)
        self.summary = tk.StringVar(value="未连接。请先接好飞控USB与舵机电源。")
        ttk.Label(frame, textvariable=self.summary, wraplength=780).pack(fill="x", pady=8)
        self.output = tk.Text(frame, height=19, wrap="word", font=("Consolas", 10))
        self.output.pack(fill="both", expand=True)
        ttk.Label(frame, text="启动按钮会实际自动起飞。关闭窗口不会停止任务；停止按钮交回飞手。录像保存在 Nano 的 log/ 目录。", wraplength=780).pack(anchor="w", pady=(8, 0))
        root.protocol("WM_DELETE_WINDOW", self.close)
        root.after(2000, self.poll)

    def log(self, text):
        self.output.insert("end", text + "\n")
        self.output.see("end")

    def invalidate(self):
        self.prepared = None
        self.start.config(state="disabled")

    def connect(self):
        host, user, password = self.host.get().strip(), self.user.get().strip(), self.password.get()
        self.invalidate()
        def work():
            client = paramiko.SSHClient()
            client.load_system_host_keys()
            known = Path.home() / ".ssh/known_hosts"
            if known.exists():
                client.load_host_keys(str(known))
            try:
                client.connect(host, username=user, password=password or None, timeout=10, banner_timeout=20)
                _, stdout, stderr = client.exec_command("hostname", timeout=5)
                name = stdout.read().decode().strip()
                if stdout.channel.recv_exit_status() != 0:
                    raise RuntimeError("远端身份检查失败")
                if self.client is not None:
                    self.client.close()
                self.client = client
                self.root.after(0, lambda: self.summary.set(f"已连接 {user}@{host} / {name}；可先打开画面，再做检查。"))
            except Exception as error:
                client.close()
                self.root.after(0, self.log, "连接失败：" + str(error))
        threading.Thread(target=work, daemon=True).start()

    def action(self, action):
        if self.client is None:
            messagebox.showinfo("尚未连接", "先输入SSH账号密码并连接。")
            return
        if self.busy and action != "stop":
            return
        route = self.route.get()
        if action == "run" and self.prepared != route:
            messagebox.showinfo("先做检查", "本条路线的起飞前检查尚未通过。")
            return
        if action in ("run", "stop") or "-" in action:
            self.invalidate()
        self.busy = True
        self.log(f"{action} / 路线{route}")
        if action in ("vision", "run"):
            webbrowser.open(f"http://{self.host.get().strip()}:8088")
        def work():
            try:
                _, stdout, stderr = self.client.exec_command(remote_command(action, route), timeout=120)
                text = stdout.read().decode("utf-8", "replace")
                errors = stderr.read().decode("utf-8", "replace")
                code = stdout.channel.recv_exit_status()
                self.root.after(0, self.finished, action, route, code, text + errors)
            except Exception as error:
                self.root.after(0, self.finished, action, route, 2, str(error))
        threading.Thread(target=work, daemon=True).start()

    def finished(self, action, route, code, output):
        self.busy = False
        self.log(output + f"\n退出码={code}")
        if action == "check" and code == 0 and self.route.get() == route:
            self.prepared = route
            self.start.config(state="normal")
            self.summary.set("检查通过。确认净空与飞手就位后，点击“启动飞行”。")
        elif code != 0:
            self.invalidate()
            self.summary.set("操作未通过，请查看原因；如果正在飞行，请由飞手接管。")

    def poll(self):
        if self.client is not None and not self.busy:
            def work():
                try:
                    _, out, _ = self.client.exec_command(remote_command("status"), timeout=3)
                    status = json.loads(out.read().decode())
                    summary = f"阶段 {status.get('phase')} / {status.get('task_phase')} | 已完成任务区 {status.get('completed_task_count',0)}/2 | {status.get('mission_elapsed_s',0):.1f}s"
                    self.root.after(0, self.summary.set, summary)
                except Exception:
                    pass
            threading.Thread(target=work, daemon=True).start()
        self.root.after(2000, self.poll)

    def close(self):
        if self.client is not None:
            self.client.close()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    Console(root)
    root.mainloop()
