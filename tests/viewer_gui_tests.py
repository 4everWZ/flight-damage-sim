"""Desktop Tk integration: resize/redraw, timer shutdown and child cleanup."""
import sys
import unittest
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from live_client import CoreClient
from viewer import Viewer


class WindowLifecycleTests(unittest.TestCase):
    def test_close_stops_timer_and_core(self):
        with CoreClient() as core:
            viewer=Viewer(core,plt)
            window=viewer.fig.canvas.manager.window
            window.withdraw()
            frames=[]
            try:
                viewer.enable_blit()
                viewer.timer=viewer.fig.canvas.new_timer(interval=33)

                def frame():
                    if viewer.closed:return False
                    viewer.display_frame()
                    frames.append(core.state['tick'])
                    if len(frames)==2:
                        window.geometry('1000x700')
                        viewer.fig.canvas.draw_idle()
                    if len(frames)==6:plt.close(viewer.fig)

                viewer.timer.add_callback(frame)
                viewer.timer.start()
                window.after(10000,lambda:plt.close(viewer.fig))
                plt.show()
                self.assertGreaterEqual(len(frames),6)
                self.assertTrue(viewer.closed)
                self.assertIsNotNone(core.process.poll())
                self.assertIsNone(viewer.timer._timer)
                self.assertFalse(plt.fignum_exists(viewer.fig.number))
            finally:
                viewer.close()
                plt.close(viewer.fig)


if __name__=='__main__':unittest.main()
