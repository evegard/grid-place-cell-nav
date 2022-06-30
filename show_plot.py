#!/usr/bin/env python2

# Navigating with grid and place cells in cluttered environments
# Edvardsen et al. (2020). Hippocampus, 30(3), 220-232.
#
# Licensed under the EUPL-1.2-or-later.
# Copyright (c) 2019 NTNU - Norwegian University of Science and Technology.
# Author: Vegard Edvardsen (https://github.com/evegard).

import argparse
import os
import PIL.Image
import PIL.ImageTk
import Queue
import StringIO
import subprocess
import sys
import threading
import Tkinter

parser = argparse.ArgumentParser()
parser.add_argument('--pipe', default='./plot_pipe')
parser.add_argument('--src-dir', default=None)
parser.add_argument('--png-dir', default=None)
parser.add_argument('--drop-frames', action='store_true')
parser.add_argument('--headless', action='store_true')
parser.add_argument('--lite-plot', action='store_true')
parser.add_argument('--verbose', '-v', action='count')
args = parser.parse_args()

print 'Rendering plots from \'%s\'' % args.pipe

def debug(string):
    if args.verbose > 0:
        print string

if not args.headless:
    debug('Main thread: Starting Tk')
    tk = Tkinter.Tk()
    tk.title('Plot viewer')
    debug('Main thread: Creating initial widget')
    widget = Tkinter.Label(tk, text='Waiting for plot data from \'%s\'' % args.pipe,
        padx=40, pady=40, font=15)
    widget.pack()
debug('Main thread: Creating plot queue and show queue')
plot_queue = Queue.Queue()
show_queue = Queue.Queue()
if not args.headless:
    debug('Main thread: Creating internal pipe')
    (tk_trigger_read, tk_trigger_write) = os.pipe()
    debug('Main thread: Setting up internal pipe callback')
    currently_shown_image = None
    def tk_trigger_callback(*args):
        debug('Main thread: In internal pipe callback; fetching image and updating UI')
        os.read(tk_trigger_read, 1)
        widget['text'] = ''
        global currently_shown_image
        currently_shown_image = show_queue.get()
        widget['image'] = currently_shown_image
        widget.pack()
        debug('Main thread: Exiting internal pipe callback')
    tk.createfilehandler(tk_trigger_read, Tkinter.READABLE, tk_trigger_callback)
debug('Main thread: Spawning pipe thread')
class PipeThread(threading.Thread):
    def run(self):
        debug('Pipe thread: Entering infinite loop')
        while True:
            debug('Pipe thread: Opening pipe')
            with open(args.pipe, 'r') as plot_pipe:
                debug('Pipe thread: Entering read loop')
                lines = []
                for line in plot_pipe:
                    if args.verbose > 1:
                        sys.stdout.write('.')
                        sys.stdout.flush()
                    lines.append(line)
                    if line == 'quit;\n':
                        debug('Pipe thread: Found terminating line, submitting data to plot queue')
                        plot_queue.put(''.join(lines))
                        lines = []
                plot_pipe.close()
                debug('Pipe thread: Got EOF, closing pipe')
                if args.headless:
                    plot_queue.put(None)
                    return
pipe_thread = PipeThread()
pipe_thread.daemon = True
pipe_thread.start()
debug('Main thread: Spawning plot thread')
class PlotThread(threading.Thread):
    def run(self):
        debug('Plot thread: Entering infinite loop')
        plot_number = 0
        while True:
            debug('Plot thread: Waiting for data from plot queue')
            plot_data = plot_queue.get()
            if args.headless and plot_data is None:
                return
            debug('Plot thread: Checking number of waiting items')
            sys.stdout.write('%d ' % plot_queue.qsize())
            sys.stdout.flush()
            if args.drop_frames and plot_queue.qsize() > 10:
                continue
            plot_number += 1
            debug('Plot thread: Possibly writing source file')
            if args.src_dir is not None:
                with open('%s/plot%d.gpi' % (args.src_dir, plot_number), 'w') as src_file:
                    src_file.write(plot_data)
            if args.headless:
                continue
            debug('Plot thread: Opening gnuplot')
            gnuplot_process = subprocess.Popen([ 'gnuplot' ],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE)
            debug('Plot thread: Writing to gnuplot and reading output')
            (image_data, _) = gnuplot_process.communicate(plot_data)
            debug('Plot thread: Possibly cropping image file')
            if args.lite_plot:
                crop_process = subprocess.Popen(
                    [ 'convert', '-crop', '1200x780+0+0', '-', '-' ],
                    stdin=subprocess.PIPE,
                    stdout=subprocess.PIPE)
                (image_data, _) = crop_process.communicate(image_data)
            debug('Plot thread: Possibly writing image file')
            if args.png_dir is not None:
                with open('%s/plot%d.png' % (args.png_dir, plot_number), 'w') as png_file:
                    png_file.write(image_data)
            debug('Plot thread: Creating image from gnuplot output')
            image = PIL.Image.open(StringIO.StringIO(image_data))
            debug('Plot thread: Creating Tk-compatible image')
            image_tk = PIL.ImageTk.PhotoImage(image)
            debug('Plot thread: Submitting image to show queue')
            show_queue.put(image_tk)
            debug('Plot thread: Writing to internal pipe')
            os.write(tk_trigger_write, ' ')
plot_thread = PlotThread()
plot_thread.daemon = True
plot_thread.start()
if not args.headless:
    debug('Main thread: Entering main loop')
    tk.mainloop()
else:
    plot_thread.join()
