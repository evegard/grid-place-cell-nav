# Navigating with grid and place cells in cluttered environments
# Edvardsen et al. (2020). Hippocampus, 30(3), 220-232.
#
# Licensed under the EUPL-1.2-or-later.
# Copyright (c) 2019 NTNU - Norwegian University of Science and Technology.
# Author: Vegard Edvardsen (https://github.com/evegard).

TARGETS += ratnav
OBJS += main.o
OBJS += network.o
OBJS += numerical.o
OBJS += plot.o
OBJS += mec.o
OBJS += mecdiff.o
OBJS += motor.o
OBJS += model.o
OBJS += simulation.o
OBJS += arena.o
OBJS += graph.o
OBJS += agent.o
OBJS += polar.o
OBJS += ui.o

DEFS += -D_POSIX_C_SOURCE=200112L
FEATURES += --std=c++11 -ffast-math -mavx -lrt

CXXFLAGS += $(DEFS) $(FEATURES) $(LIBS) -O3 -g

.PHONY: all clean show-plot show-plot-and-generate-movie
all: $(TARGETS)
clean:
	$(RM) $(TARGETS) $(OBJS)
show-plot:
	./show_plot.py
show-plot-and-generate-movie:
	mkdir -p output
	$(RM) output/*
	./show_plot.py --png-dir output
	ffmpeg -framerate 20 -i output/plot%d.png -vb 2M output/movie.mp4
	xdg-open output/movie.mp4

ratnav: $(OBJS)
	$(CXX) -o $@ $^ $(CXXFLAGS)
