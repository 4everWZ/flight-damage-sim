"""Real-process cleanup regressions; run with the viewer virtual environment."""
import subprocess
import sys
import unittest
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from live_client import CoreClient


class CleanupTests(unittest.TestCase):
    def test_live_core_exits_and_close_is_idempotent(self):
        client=CoreClient()
        client.close()
        client.close()
        self.assertIsNotNone(client.process.poll())
        self.assertTrue(client.process.stdin.closed)
        self.assertTrue(client.process.stdout.closed)

    def test_unresponsive_child_is_terminated(self):
        client=CoreClient.__new__(CoreClient)
        client.process=subprocess.Popen(
            [sys.executable,'-u','-c','import time; print("ready"); time.sleep(60)'],
            stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
        try:
            self.assertEqual(client.process.stdout.readline().strip(),'ready')
            client.close()
            self.assertIsNotNone(client.process.poll())
        finally:
            if client.process.poll() is None:
                client.process.kill()
                client.process.wait()
            client.close()

    def test_already_exited_child(self):
        client=CoreClient()
        client.process.kill()
        client.process.wait()
        client.close()
        self.assertTrue(client.process.stdout.closed)


if __name__=='__main__':unittest.main()
