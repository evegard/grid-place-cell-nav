#!/usr/bin/env python2

# Navigating with grid and place cells in cluttered environments
# Edvardsen et al. (2020). Hippocampus, 30(3), 220-232.
#
# Licensed under the EUPL-1.2-or-later.
# Copyright (c) 2019 NTNU - Norwegian University of Science and Technology.
# Author: Vegard Edvardsen (https://github.com/evegard).

import argparse
import math
import os.path
import re
import subprocess
import sys
import xml.etree.ElementTree

parser = argparse.ArgumentParser()
parser.add_argument('script', type=argparse.FileType('r'))
parser.add_argument('current_trajectory', type=int, nargs='?', default=1) # 1-indexed!
parser.add_argument('total_trajectories', type=int, nargs='?', default=1)
args = parser.parse_args()

class Converter:
    namespaces = {
        'svg': 'http://www.w3.org/2000/svg',
        'inkscape': 'http://www.inkscape.org/namespaces/inkscape',
    }

    def __init__(self, args):
        self.args = args
        self.svg_root = None
        self.svg_scale = 1.0

    def load_svg(self, svg_filename, svg_scale):
        env_directory = os.path.dirname(self.args.script.name)
        full_svg_filename = os.path.join(env_directory, svg_filename)
        original_svg_file = open(full_svg_filename, 'r')

        path_ids = [ element.get('id') for element in
            xml.etree.ElementTree.parse(original_svg_file).getroot() \
                .findall('.//svg:path[@id]', self.namespaces) ]
        params = [ '/usr/share/inkscape/extensions/flatten.py' ]
        params += [ '--flatness', '0.1' ]
        params += sum([ [ '--id', path_id ] for path_id in path_ids ], [])
        params += [ full_svg_filename ]

        original_svg_file.close()
        flatten_process = subprocess.Popen(params, stdout=subprocess.PIPE)
        flatten_output = flatten_process.communicate()

        processed_svg_root = xml.etree.ElementTree.fromstring(flatten_output[0])
        self.svg_root = processed_svg_root
        self.svg_scale = svg_scale

    def transform_coordinates(self, svg_x, svg_y):
        return (svg_x * self.svg_scale, -svg_y * self.svg_scale)

    def parse_path_from_element(self, element):
        element_d = element.get('d')
        element_d = re.sub(',', ' ', element_d)
        element_d = re.sub('([MmZzLl])', ' \\1 ', element_d)
        tokens = element_d.split()
        tokens = filter(lambda x: x != '', tokens)
        path = []
        prev = (0.0, 0.0)
        relative_mode = False
        while len(tokens) > 0:
            token = tokens.pop(0)
            if token in [ 'M', 'L' ]:
                relative_mode = False
            elif token in [ 'm', 'l' ]:
                relative_mode = True
            elif token in [ 'Z', 'z' ]:
                assert(len(tokens) == 0)
                path.append(path[0])
            else:
                x = float(token)
                y = float(tokens.pop(0))
                if relative_mode:
                    (x, y) = (x + prev[0], y + prev[1])
                prev = (x, y)
                path.append(self.transform_coordinates(x, y))
        return path

    def get_layer_by_label(self, layer_label):
        element_filter = './svg:g[@inkscape:label=\'%s\']' % layer_label
        elements = self.svg_root.findall(element_filter, self.namespaces)
        if len(elements) != 1:
            raise Exception('Could not find layer with label \'%s\'' % layer_label)
        return elements[0]

    def get_paths_by_layer_label(self, layer_label):
        layer = self.get_layer_by_label(layer_label)
        return [ self.parse_path_from_element(element) for element in
            layer.findall('svg:path', self.namespaces) ]

    def get_label_commands_by_layer_label(self, layer_label):
        layer = self.get_layer_by_label(layer_label)
        commands = []
        for element in layer.findall('svg:text', self.namespaces):
            coords = self.transform_coordinates(
                float(element.get('x')), float(element.get('y')))
            label_text = element.find('svg:tspan', self.namespaces).text
            commands.append('add-label %f %f %s' % (coords[0], coords[1], label_text))
        return '\n'.join(commands)

    def get_path_by_id(self, path_id):
        element_filter = './/svg:path[@id=\'%s\']' % path_id
        elements = self.svg_root.findall(element_filter, self.namespaces)
        if len(elements) != 1:
            raise Exception('Could not find path with ID \'%s\'' % path_id)
        return self.parse_path_from_element(elements[0])

    def calculate_segment_length(self, path, segment):
        return ((path[segment + 1][0] - path[segment][0]) ** 2 +
                (path[segment + 1][1] - path[segment][1]) ** 2) ** 0.5

    def calculate_path_length(self, path):
        return sum([
            self.calculate_segment_length(path, segment)
            for segment in range(len(path) - 1) ])

    def generate_sampled_trajectory(self, path, sample_length):
        cumulative_path_length = 0.0
        trajectory = [ path[0] ]
        next_segment = 0
        # Number of coords: len(path)
        # Number of segments: len(path) - 1
        # Number of segments we want to add directly: len(path) - 2
        # Last valid segment ID: len(path) - 3
        while next_segment < len(path) - 2:
            next_segment_length = self.calculate_segment_length(path, next_segment)
            if cumulative_path_length + next_segment_length > sample_length:
                break
            cumulative_path_length += next_segment_length
            trajectory.append(path[next_segment + 1])
            next_segment += 1
        remaining_length = sample_length - cumulative_path_length
        segment_start = path[next_segment]
        segment_end = path[next_segment + 1]
        segment_dx = segment_end[0] - segment_start[0]
        segment_dy = segment_end[1] - segment_start[1]
        segment_length = self.calculate_segment_length(path, next_segment)
        final_x = segment_start[0] + segment_dx * 1.0 * remaining_length / segment_length
        final_y = segment_start[1] + segment_dy * 1.0 * remaining_length / segment_length
        trajectory.append((final_x, final_y))
        return trajectory

    def convert_paths_to_multipolygon_wkt(self, paths):
        return 'MULTIPOLYGON(' + ','.join([
            '((' + ','.join([ '%f %f' % (x, y) for (x, y) in path ]) + '))'
            for path in paths
        ]) + ')'

    def convert_path_to_goto_commands(self, path):
        return '\n'.join([ 'goto %f %f' % (x, y) for (x, y) in path ])

    def process_script(self):
        for line in self.args.script:
            if line[0] == '#':
                continue
            tokens = line.split()
            command = tokens[0]
            if command in [ 'trigger-reward', 'seek-reward',
                    'place-agent', 'set-trial-phase', 'goto',
                    'set-title', 'set-origin', 'set-fence',
                    'add-label', 'set-scale-bars' ]:
                print ' '.join(tokens)
            elif command == 'load-svg':
                (svg_filename, svg_scale) = tokens[1:3]
                svg_scale = float(svg_scale)
                self.load_svg(svg_filename, svg_scale)
                print 'set-arena-size', (200 * svg_scale)
            elif command == 'set-arena-from-svg-layer':
                layer_label = tokens[1]
                paths = self.get_paths_by_layer_label(layer_label)
                paths_as_wkt = self.convert_paths_to_multipolygon_wkt(paths)
                print 'set-arena', paths_as_wkt
            elif command == 'add-labels-from-svg-layer':
                layer_label = tokens[1]
                print self.get_label_commands_by_layer_label(layer_label)
            elif command == 'follow-trajectory-from-svg-path':
                path_id = tokens[1]
                path = self.get_path_by_id(path_id)
                print self.convert_path_to_goto_commands(path)
            elif command == 'sample-perimeter-from-svg-path':
                assert(self.args.total_trajectories > 0)
                assert(self.args.current_trajectory > 0)
                assert(self.args.current_trajectory <= self.args.total_trajectories)
                path_id = tokens[1]
                path = self.get_path_by_id(path_id)
                path_length = self.calculate_path_length(path)
                ratio = 1.0 * (self.args.current_trajectory - 1) / self.args.total_trajectories
                sample_length = path_length * ratio
                sampled_trajectory = self.generate_sampled_trajectory(path, sample_length)
                print self.convert_path_to_goto_commands(sampled_trajectory)
            elif command == 'place-agent-at-start-of-svg-path':
                path_id = tokens[1]
                path = self.get_path_by_id(path_id)
                print 'place-agent %f %f %f' % (path[0][0], path[0][1], \
                    math.atan2(path[1][1] - path[0][1], path[1][0] - path[0][0]))
            elif command == 'set-fence-from-svg-path':
                path_id = tokens[1]
                path = self.get_path_by_id(path_id)
                path_as_wkt = self.convert_paths_to_multipolygon_wkt([ path ])
                print 'set-fence', path_id, path_as_wkt
            else:
                raise Exception('Unknown command \'%s\'' % tokens[0])

Converter(args).process_script()
