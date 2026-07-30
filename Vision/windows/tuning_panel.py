"""Optional Tk tuning panel for live MaixCAM vision parameters."""


TUNING_WINDOW = "滚球视觉调参（只影响识别，不控制电机）"


class TuningPanel:
    FIELDS = (
        {
            "key": "target_position",
            "group": "常用参数",
            "title": "目标停球位置",
            "minimum": 5,
            "maximum": 95,
            "scale": 100.0,
            "unit": "%",
            "description": "管道左端为 0%，右端为 100%；只改变目标点。",
        },
        {
            "key": "position_alpha",
            "group": "常用参数",
            "title": "位置跟随速度",
            "minimum": 5,
            "maximum": 100,
            "scale": 100.0,
            "unit": "%",
            "description": "越大跟得越快但更抖；越小更稳但会滞后。",
        },
        {
            "key": "velocity_beta",
            "group": "常用参数",
            "title": "速度修正强度",
            "minimum": 0,
            "maximum": 100,
            "scale": 100.0,
            "unit": "%",
            "description": "越大越快修正速度估计，也更容易放大噪声。",
        },
        {
            "key": "coast_frames",
            "group": "常用参数",
            "title": "短暂丢球后继续预测",
            "minimum": 0,
            "maximum": 15,
            "scale": 1.0,
            "unit": " 帧",
            "description": "遮挡或模糊时还能预测几帧；过大会使用过期位置。",
        },
        {
            "key": "lateral_alpha",
            "group": "识别容错",
            "title": "横向位置跟随速度",
            "minimum": 5,
            "maximum": 100,
            "scale": 100.0,
            "unit": "%",
            "description": "控制垂直于管道方向的平滑，一般无需频繁调整。",
        },
        {
            "key": "max_axis_distance_px",
            "group": "识别容错",
            "title": "中心线上方允许距离",
            "minimum": 5,
            "maximum": 80,
            "scale": 1.0,
            "unit": " 像素",
            "description": "适应管道向上移动；越大也越容易接纳管道外背景。",
        },
        {
            "key": "max_below_axis_distance_px",
            "group": "识别容错",
            "title": "中心线下方允许距离",
            "minimum": 3,
            "maximum": 80,
            "scale": 1.0,
            "unit": " 像素",
            "description": "当前背景假点多在管道下方，建议明显小于上方范围。",
        },
        {
            "key": "max_frame_jump_px",
            "group": "识别容错",
            "title": "钢球单帧最大移动距离",
            "minimum": 10,
            "maximum": 240,
            "scale": 1.0,
            "unit": " 像素",
            "description": "高速滚动需要更大；过大会允许轨迹跳到假目标。",
        },
        {
            "key": "local_search_width_px",
            "group": "识别容错",
            "title": "钢球附近搜索宽度",
            "minimum": 40,
            "maximum": 470,
            "scale": 1.0,
            "unit": " 像素",
            "description": "越小越快；高速、震动或易丢球时适当增大。",
        },
        {
            "key": "acquire_min_quality",
            "group": "防止误识别",
            "title": "重新找到球所需的最低可信度",
            "minimum": 0,
            "maximum": 200,
            "scale": 1.0,
            "unit": " 分",
            "description": "越高越不易锁到反光或管口；过高会延迟丢球后的恢复。",
        },
        {
            "key": "track_min_quality",
            "group": "防止误识别",
            "title": "稳定跟踪时的最低可信度",
            "minimum": 0,
            "maximum": 200,
            "scale": 1.0,
            "unit": " 分",
            "description": "过滤单帧小反光；低于门槛时改用短暂预测，不输出假测量。",
        },
        {
            "key": "acquire_position_margin",
            "group": "防止误识别",
            "title": "重新找球时允许超出端点",
            "minimum": 0,
            "maximum": 10,
            "scale": 100.0,
            "unit": "%",
            "description": "真实钢球可到旧轴终点外约 7%；当前使用 8%。",
        },
        {
            "key": "track_position_margin",
            "group": "防止误识别",
            "title": "已跟踪时允许超出管道端点",
            "minimum": 0,
            "maximum": 8,
            "scale": 100.0,
            "unit": "%",
            "description": "稳定轨迹的端点容差；应覆盖真实行程但不要继续放大。",
        },
        {
            "key": "acquire_endpoint_inset",
            "group": "端点与圆检测",
            "title": "首次找球时端部禁区",
            "minimum": 0,
            "maximum": 12,
            "scale": 100.0,
            "unit": "%",
            "description": "两端各排除多少管长；越大越防反光，过大会漏掉端部真球。",
        },
        {
            "key": "track_endpoint_inset",
            "group": "端点与圆检测",
            "title": "稳定跟踪时端部禁区",
            "minimum": 0,
            "maximum": 8,
            "scale": 100.0,
            "unit": "%",
            "description": "已锁定后使用的较小禁区；当前车载标定为 1.5%。",
        },
        {
            "key": "circle_threshold",
            "group": "端点与圆检测",
            "title": "钢球圆检测严格度",
            "minimum": 100,
            "maximum": 5000,
            "scale": 1.0,
            "unit": "",
            "description": "越高候选越少、误圆更少；过高会漏掉反光或模糊钢球。",
        },
        {
            "key": "circle_min_radius",
            "group": "端点与圆检测",
            "title": "钢球最小半径",
            "minimum": 6,
            "maximum": 24,
            "scale": 1.0,
            "unit": " 像素",
            "description": "排除比钢球更小的固定螺丝；更换高度后需重新测量。",
        },
        {
            "key": "circle_max_radius",
            "group": "端点与圆检测",
            "title": "钢球最大半径",
            "minimum": 8,
            "maximum": 32,
            "scale": 1.0,
            "unit": " 像素",
            "description": "排除大块支架和重叠反光；必须大于最小半径。",
        },
    )

    def __init__(self, config, sync_offset_ms=0.0):
        import tkinter as tk
        from tkinter import ttk

        self._tk = tk
        self.root = tk.Tk()
        self.root.title(TUNING_WINDOW)
        self.root.geometry("720x650")
        self.root.protocol("WM_DELETE_WINDOW", self._request_close)
        self.root.option_add("*Font", ("Microsoft YaHei UI", 10))
        self._closed = False
        self._apply_requested = False
        self._variables = {}
        self._value_labels = {}
        self.applied_config = dict(config or {})

        title = ttk.Label(
            self.root,
            text="滚球视觉调参",
            font=("Microsoft YaHei UI", 15, "bold"),
        )
        title.pack(anchor="w", padx=16, pady=(12, 2))
        ttk.Label(
            self.root,
            text=(
                "这里只调整钢球识别与跟踪，不会向步进电机发送任何指令。"
                " 调整后点击“应用到 MaixCAM”。"
            ),
            foreground="#555555",
            wraplength=675,
        ).pack(anchor="w", padx=16, pady=(0, 8))

        notebook = ttk.Notebook(self.root)
        notebook.pack(fill="both", expand=True, padx=14)
        group_frames = {}
        for group in (
            "常用参数",
            "识别容错",
            "防止误识别",
            "端点与圆检测",
        ):
            frame = ttk.Frame(notebook, padding=10)
            notebook.add(frame, text=group)
            group_frames[group] = frame

        for spec in self.FIELDS:
            parent = group_frames[spec["group"]]
            row = len(parent.grid_slaves()) // 3
            key = spec["key"]
            initial = float(self.applied_config.get(key, 0))
            display_value = initial * float(spec["scale"])
            variable = tk.DoubleVar(value=display_value)
            self._variables[key] = variable

            ttk.Label(
                parent,
                text=spec["title"],
                font=("Microsoft YaHei UI", 10, "bold"),
            ).grid(row=row * 3, column=0, sticky="w", pady=(5, 0))
            value_label = ttk.Label(parent, width=12, anchor="e")
            value_label.grid(
                row=row * 3, column=1, sticky="e", pady=(5, 0)
            )
            self._value_labels[key] = value_label

            slider = tk.Scale(
                parent,
                from_=spec["minimum"],
                to=spec["maximum"],
                resolution=1,
                orient=tk.HORIZONTAL,
                showvalue=False,
                variable=variable,
                length=615,
                highlightthickness=0,
                command=lambda _value, field=key: self._refresh_value(
                    field
                ),
            )
            slider.grid(
                row=row * 3 + 1,
                column=0,
                columnspan=2,
                sticky="ew",
            )
            ttk.Label(
                parent,
                text=spec["description"],
                foreground="#666666",
            ).grid(
                row=row * 3 + 2,
                column=0,
                columnspan=2,
                sticky="w",
                pady=(0, 4),
            )
            self._refresh_value(key)
        for parent in group_frames.values():
            parent.columnconfigure(0, weight=1)

        sync_frame = ttk.LabelFrame(
            self.root,
            text="画面与标注",
            padding=(10, 4),
        )
        sync_frame.pack(fill="x", padx=14, pady=(8, 4))
        self._sync_variable = tk.DoubleVar(value=float(sync_offset_ms))
        ttk.Label(
            sync_frame,
            text="时间微调：正常保持 0 ms；只有自动同步后仍有固定偏差才调整。",
        ).grid(row=0, column=0, sticky="w")
        self._sync_label = ttk.Label(sync_frame, width=10, anchor="e")
        self._sync_label.grid(row=0, column=1, sticky="e")
        tk.Scale(
            sync_frame,
            from_=-300,
            to=300,
            resolution=5,
            orient=tk.HORIZONTAL,
            showvalue=False,
            variable=self._sync_variable,
            highlightthickness=0,
            command=lambda _value: self._refresh_sync_value(),
        ).grid(row=1, column=0, columnspan=2, sticky="ew")
        sync_frame.columnconfigure(0, weight=1)
        self._refresh_sync_value()

        button_row = ttk.Frame(self.root)
        button_row.pack(fill="x", padx=14, pady=(6, 4))
        ttk.Button(
            button_row,
            text="应用到 MaixCAM",
            command=self._request_apply,
        ).pack(side="left")
        ttk.Button(
            button_row,
            text="恢复设备当前值",
            command=self.reset_to_applied,
        ).pack(side="left", padx=8)
        self._status_label = ttk.Label(
            button_row,
            text="尚未修改",
            foreground="#555555",
        )
        self._status_label.pack(side="right")

    def _field(self, key):
        return next(spec for spec in self.FIELDS if spec["key"] == key)

    def _refresh_value(self, key):
        spec = self._field(key)
        raw = float(self._variables[key].get())
        text = "{}{}".format(int(round(raw)), spec["unit"])
        self._value_labels[key].configure(text=text)

    def _refresh_sync_value(self):
        self._sync_label.configure(
            text="{:+.0f} ms".format(self._sync_variable.get())
        )

    def _request_apply(self):
        self._apply_requested = True
        self.set_status("正在等待发送…")

    def _request_close(self):
        self._closed = True

    def poll(self):
        if self._closed:
            return False
        try:
            self.root.update_idletasks()
            self.root.update()
            return True
        except self._tk.TclError:
            self._closed = True
            return False

    def consume_apply_request(self):
        requested = self._apply_requested
        self._apply_requested = False
        return requested

    def read(self):
        params = {}
        for spec in self.FIELDS:
            raw = float(self._variables[spec["key"]].get())
            value = raw / float(spec["scale"])
            if spec["scale"] == 1.0:
                value = int(round(value))
            params[spec["key"]] = value
        return params

    def sync_offset_ms(self):
        return float(self._sync_variable.get())

    def set_sync_offset(self, value):
        self._sync_variable.set(float(value))
        self._refresh_sync_value()

    def reset_to_applied(self):
        for spec in self.FIELDS:
            key = spec["key"]
            if key not in self.applied_config:
                continue
            self._variables[key].set(
                float(self.applied_config[key]) * float(spec["scale"])
            )
            self._refresh_value(key)
        self.set_status("已恢复到设备当前值")

    def acknowledge(self, config):
        self.applied_config = dict(config or self.applied_config)
        self.set_status("设备已确认应用", ok=True)

    def set_status(self, text, ok=None):
        color = "#555555"
        if ok is True:
            color = "#16803a"
        elif ok is False:
            color = "#c62828"
        self._status_label.configure(text=str(text), foreground=color)

    def close(self):
        try:
            self.root.destroy()
        except self._tk.TclError:
            pass
