import os
import sys
import argparse

env_path = os.path.join(os.path.dirname(__file__), '..')
if env_path not in sys.path:
    sys.path.append(env_path)

from pytracking.evaluation import Tracker


def run_video(tracker_name, tracker_param, videofile, optional_box=None, debug=None):
    """Run the tracker on your webcam.
    args:
        tracker_name: Name of tracking method.
        tracker_param: Name of parameter file.
        debug: Debug level.
    """
    tracker = Tracker(tracker_name, tracker_param)
    tracker.run_video(videofilepath=videofile, optional_box=optional_box, debug=debug)

def main():
    parser = argparse.ArgumentParser(description='Run the tracker on your webcam.')
    parser.add_argument('--tracker_name', type=str, default='eco', help='Name of tracking method.')
    parser.add_argument('--tracker_param', type=str, default='default', help='Name of parameter file.')
    parser.add_argument('--videofile', type=str, default='E:\BaiduNetdiskDownload\\net\\net2.mp4', help='path to a video file.')
    parser.add_argument('--optional_box', default=None, help='optional_box with format x,y,w,h.')
    parser.add_argument('--debug', type=int, default=0, help='Debug level.')

    args = parser.parse_args()

    run_video(args.tracker_name, args.tracker_param, args.videofile, args.optional_box, args.debug)


if __name__ == '__main__':
    main()
