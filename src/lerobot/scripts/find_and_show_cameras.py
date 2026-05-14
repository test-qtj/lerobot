import cv2
import time
import threading


class CameraWorker:
    def __init__(self, index, width=640, height=480, fps=90):
        self.index = index
        self.width = width
        self.height = height
        self.fps = fps

        self.cap = None
        self.frame = None
        self.ok = False
        self.running = False

        self.lock = threading.Lock()
        self.thread = None

    def open(self):
        self.cap = cv2.VideoCapture(self.index, cv2.CAP_V4L2)

        if not self.cap.isOpened():
            return False

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

        # 尽量减少缓冲延迟
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        time.sleep(0.2)

        ok, frame = self.cap.read()

        if not ok or frame is None:
            self.cap.release()
            self.cap = None
            return False

        with self.lock:
            self.frame = frame
            self.ok = True

        real_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        real_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        real_fps = self.cap.get(cv2.CAP_PROP_FPS)
        fourcc = int(self.cap.get(cv2.CAP_PROP_FOURCC))
        fourcc_str = "".join(chr((fourcc >> 8 * i) & 0xFF) for i in range(4))

        print(f"[OK] Camera index {self.index}")
        print(f"     Resolution: {real_w} x {real_h}")
        print(f"     FPS: {real_fps}")
        print(f"     FOURCC: {fourcc_str}")

        return True

    def start(self):
        if self.cap is None:
            raise RuntimeError(f"Camera {self.index} is not opened")

        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self):
        while self.running:
            ok, frame = self.cap.read()

            if ok and frame is not None:
                with self.lock:
                    self.frame = frame
                    self.ok = True
            else:
                with self.lock:
                    self.ok = False

            # 稍微让出 CPU
            time.sleep(0.001)

    def get_frame(self):
        with self.lock:
            if self.frame is None:
                return False, None
            return self.ok, self.frame.copy()

    def stop(self):
        self.running = False

        if self.thread is not None:
            self.thread.join(timeout=1.0)

        if self.cap is not None:
            self.cap.release()
            self.cap = None


def scan_cameras(max_index=10, width=640, height=480, fps=30):
    workers = []

    print("======================================================")
    print("Scanning cameras...")
    print("======================================================")

    for index in range(max_index):
        worker = CameraWorker(
            index=index,
            width=width,
            height=height,
            fps=fps,
        )

        if worker.open():
            workers.append(worker)
        else:
            print(f"[SKIP] Camera index {index} unavailable")

    print("======================================================")
    print(f"Found {len(workers)} camera(s)")
    print("======================================================")

    return workers


def show_cameras(workers):
    if len(workers) == 0:
        print("No camera found.")
        return

    for worker in workers:
        worker.start()

    print("")
    print("Press q in any OpenCV window to quit.")
    print("Press Ctrl+C in terminal to force quit.")
    print("")

    try:
        while True:
            for worker in workers:
                ok, frame = worker.get_frame()

                if not ok or frame is None:
                    continue

                cv2.putText(
                    frame,
                    f"Camera index: {worker.index}",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.0,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )

                window_name = f"Camera {worker.index}"
                cv2.imshow(window_name, frame)

            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                print("Quit by q.")
                break

    except KeyboardInterrupt:
        print("")
        print("Quit by Ctrl+C.")

    finally:
        for worker in workers:
            worker.stop()

        cv2.destroyAllWindows()
        print("All cameras released.")


def main():
    workers = scan_cameras(
        max_index=10,
        width=640,
        height=480,
        fps=90,
    )

    show_cameras(workers)


if __name__ == "__main__":
    main()