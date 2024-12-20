#!/usr/bin/env python

class RangeSensor():
    def __init__(self, angle=[0.0,0.0,0.0], ranges=[0.0,0.0]):
        self.angle_min = angle[0]
        self.angle_max = angle[1]
        self.angle_increment = angle[2]
        self.range_min = ranges[0]
        self.range_max = ranges[1]
        self.data_distancies = []
        self.data_intensities = []
    def update_ranges(self,distancies,intensities):
        self.data_distancies = distancies
        self.data_intensities = intensities
    def get_ranges(self):
        return self.data_distancies
    def get_ranges_with_angle(self):
        angle = self.angle_min
        ranges_with_angle = []
        for dist in self.data_distancies:
            angle += self.angle_increment
            if dist != self.range_max:
                ranges_with_angle.append( (dist,angle) )
        return ranges_with_angle
