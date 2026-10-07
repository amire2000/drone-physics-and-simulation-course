"""Small external Tk controls shared by interactive PyBullet examples."""

class TkSimulationControls:
    """Own Start, Pause, Restart, and Quit controls for a physics loop."""

    def __init__(self, title: str) -> None:
        import tkinter as tk

        self._root = tk.Tk()
        self._root.title(title)
        self._running = False
        self._restart_requested = False
        self._closed = False
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
        self._status.set("Restart requested")

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
