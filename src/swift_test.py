import sys

from PySide6.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
from PySide6.QtWebEngineWidgets import QWebEngineView
from PySide6.QtCore import QUrl, QTimer
import roboticstoolbox as rtb
from swift import Swift


class SwiftEmbeddable(Swift):
    """
    Thin subclass of Swift that:
      - suppresses the external browser tab (browser=None)
      - exposes the running URL as self.url after launch()
    """

    def launch(self, realtime=False, headless=False, rate=60,
               browser=None, comms="websocket", **kwargs):
        """
        Intercept start_servers to capture the port numbers before they are
        consumed from inq, then expose them as self.url.
        """
        import swift.SwiftRoute as _sr
        from queue import Queue

        # Temporarily replace start_servers with a version that records ports
        _captured = {}
        _real_start_servers = _sr.start_servers

        def _capturing_start_servers(outq, inq, stop_servers,
                                     open_tab=True, browser=None,
                                     comms="websocket"):
            socket_thread, server_thread = _real_start_servers(
                outq, inq, stop_servers,
                open_tab=open_tab, browser=browser, comms=comms
            )
            # Reconstruct URL: socket_port was put into inq first, then
            # server_port. Both have already been consumed by start_servers
            # before it returns, so we recover them from the thread targets.
            #
            # SwiftSocket stores self.loop on port 53000+ (scanned from 53000).
            # SwiftServer stores server_port internally (scanned from 52000).
            # The *websocket* port is passed as socket_port arg to SwiftServer,
            # which embeds it in the redirect URL as /?{socket_port}.
            #
            # Easiest recovery: grab the port from the SwiftSocket instance
            # stored in the thread's _target closure (Thread target= kwarg).
            #
            # In Python's threading.Thread, the target is stored as
            # _target (private). The args are stored as _args.
            # SwiftSocket is instantiated inside the thread (target=SwiftSocket)
            # with args=(outq, inq, stop_servers). The socket_port is put into
            # inq *before* start_servers returns, but it has been .get()'d
            # already. However, the SwiftServer thread receives socket_port as
            # its 3rd positional arg — we can read it from server_thread._args.
            socket_port = server_thread._args[2]   # arg index 2
            # server_port: SwiftServer puts it into inq immediately; by the time
            # start_servers returns it has been .get()'d. Recover it by scanning
            # the same range SwiftServer uses (52000+) until we find one that
            # is occupied — or just read it from the HTTP server's socket.
            # Simplest: SwiftServer's TCPServer is kept alive in its thread.
            # We can't easily reach it. Instead, scan from 52000 upward.
            import socket as _socket
            for port in range(52000, 62000):
                with _socket.socket(_socket.AF_INET, _socket.SOCK_STREAM) as s:
                    if s.connect_ex(("localhost", port)) == 0:
                        server_port = port
                        break
            _captured["url"] = f"http://localhost:{server_port}/?{socket_port}"
            return socket_thread, server_thread

        _sr.start_servers = _capturing_start_servers
        try:
            # Pass browser=None to suppress the external browser tab entirely
            super().launch(realtime=realtime, headless=headless, rate=rate,
                           browser=None, comms=comms, **kwargs)
        finally:
            _sr.start_servers = _real_start_servers

        self.url = _captured.get("url", "http://localhost:52000/?53000")


class RobotGui(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Swift Robot Sim in PySide6")
        self.resize(1020, 720)

        # Launch Swift suppressing the external browser tab
        self.env = SwiftEmbeddable()
        self.env.launch()

        swift_url = self.env.url
        print(f"Swift running at: {swift_url}")

        # Embed Swift in a QWebEngineView
        self.browser = QWebEngineView()
        self.browser.setUrl(QUrl(swift_url))

        layout = QVBoxLayout()
        layout.addWidget(self.browser)
        container = QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)

        # Add a robot
        self.robot = rtb.models.Panda()
        self.env.add(self.robot)

        # Step the sim periodically so the visualizer stays live
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._step)
        self._timer.start(50)  # 50ms = 20 Hz

    def _step(self):
        self.env.step(0.05)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui = RobotGui()
    gui.show()
    sys.exit(app.exec())