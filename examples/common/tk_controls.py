"""Small external Tk controls shared by interactive PyBullet examples."""

class TkSimulationControls:
    """Own Start, Pause, Restart, and Quit controls for a physics loop."""

    def __init__(self, title: str, topic_actions: tuple[tuple[str, str], ...] = ()) -> None:
        import tkinter as tk

        self._root = tk.Tk()
        self._root.title(title)
        self._running = False
        self._restart_requested = False
        self._closed = False
        self._topic_actions = {key: False for _, key in topic_actions}
        self._status = tk.StringVar(value="Waiting for Start")
        self._root.protocol("WM_DELETE_WINDOW", self.close)

        tk.Label(self._root, text="Simulation controls").pack(padx=16, pady=(12, 6))
        tk.Label(self._root, textvariable=self._status, width=42).pack(padx=16, pady=(0, 6))
        buttons = tk.Frame(self._root)
        buttons.pack(padx=16, pady=(0, 12))
        tk.Button(buttons, text="Start", width=10, command=self.start).grid(row=0, column=0, padx=3)
        tk.Button(buttons, text="Pause", width=10, command=self.pause).grid(row=0, column=1, padx=3)
        tk.Button(buttons, text="Restart", width=10, command=self.restart).grid(row=0, column=2, padx=3)
        tk.Button(buttons, text="Quit", width=10, command=self.close).grid(row=0, column=3, padx=3)
        for column, (label, key) in enumerate(topic_actions, start=4):
            tk.Button(buttons, text=label, width=12, command=lambda action_key=key: self.toggle_action(action_key)).grid(row=0, column=column, padx=3)

    def start(self) -> None:
        """Allow the topic loop to advance physics ticks."""
        self._running = True
        self._status.set("Running")

    def pause(self) -> None:
        """Pause the topic loop without changing its simulation state."""
        self._running = False
        self._status.set("Paused")

    def pause_with_reason(self, reason: str) -> None:
        """Pause the loop and show an automatic safety reason."""
        self._running = False
        self._status.set(reason)

    def restart(self) -> None:
        """Request a topic-owned reset before the next physics tick."""
        self._running = False
        self._restart_requested = True
        self.clear_actions()
        self._status.set("Restart requested")

    def toggle_action(self, key: str) -> None:
        """Latch a topic action on or off without applying simulation physics."""
        if key not in self._topic_actions:
            raise KeyError(f"Unknown topic action: {key}")
        self._topic_actions[key] = not self._topic_actions[key]
        active = [name for name, action_key in self._topic_actions.items() if action_key]
        self._status.set(f"Active: {', '.join(active)}" if active else "No disturbance")

    def action_active(self, key: str) -> bool:
        """Return whether a named topic action is currently latched on."""
        return self._topic_actions.get(key, False)

    def clear_actions(self) -> None:
        """Clear all topic-specific latched actions."""
        for key in self._topic_actions:
            self._topic_actions[key] = False

    def poll(self) -> str:
        """Process Tk events and return ``run``, ``pause``, ``restart``, or ``exit``."""
        if self._closed:
            return "exit"
        self._root.update_idletasks()
        self._root.update()
        if self._closed:
            return "exit"
        if self._restart_requested:
            self._restart_requested = False
            return "restart"
        return "run" if self._running else "pause"

    @property
    def closed(self) -> bool:
        """Return whether the user closed the control window or chose Quit."""
        return self._closed

    def close(self) -> None:
        """Close the external controls window safely."""
        if not self._closed:
            self._closed = True
            self._root.destroy()
