#!/usr/bin/env python


class MemoryItem():
    def __init__(self):
        self.__group = 0
        self.__number = 0
        self.__curve_index = 0
        self.__time_curve = 0
        self.__time = 0
        self.__pose2D = [0,0,0]
        self.__will = 0
        self.__state = False

    def update(self, update_data):
        if "group" in update_data:
            self.__group = update_data["group"]
        if "number" in update_data:
            self.__number = update_data["number"]
        if "curve_index" in update_data:
            self.__curve_index = update_data["curve_index"]
        if "time_curve" in update_data:
            self.__time_curve = update_data["time_curve"]
        if "time" in update_data:
            self.__time = update_data["time"]
        if "pose2D" in update_data:
            self.__pose2D = update_data["pose2D"]
        if "will" in update_data:
            self.__will = update_data["will"]
        if "state" in update_data:
            self.__state = update_data["state"]

    def get(self):
        return {
            "group": self.__group,
            "number": self.__number,
            "curve_index": self.__curve_index,
            "time_curve": self.__time_curve,
            "time": self.__time,
            "pose2D": self.__pose2D,
            "will": self.__will,
            "state": self.__state
        }


class RobotMemory():
    def __init__(self):
        self.__data = [MemoryItem()]

    def reset(self):
        self.__data = [self.__data[0]]

    def update_memory_about_itself(self, i_data):
        self.__data[0].update(i_data)

    def get_memory_about_itself(self):
        return self.__data[0].get()

    def get_memory_about_others(self):
        return [other.get() for other in self.__data[1:]]

    def get_memory_about_neighbours(self,c):
        others = [other.get() for other in self.__data[1:]]
        itself = self.__data[0].get()
        dist = lambda a,b: (a[0] - b[0])**2 + (a[1] - b[0])**2
        in_range = [ dist(itself["pose2D"],other["pose2D"]) < c**2 for other in others]
        return [other for other,d in zip(others,in_range) if d]

    def get_memory(self):
        return [self.get_memory_about_itself()] + self.get_memory_about_others()

    def check_and_update_memory_about_j(self, j_data):
        if not j_data["curve_index"]:
            return
        j_in_data = False
        i_data = self.__data[0].get()
        for memory_idx in range(1,len(self.__data)):
            if self.__data[memory_idx].get()["number"] == j_data["number"]:
                j_in_data = True
                if self.__data[memory_idx].get()["time"] < j_data["time"]:
                    self.__data[memory_idx].update(j_data)
        if not j_in_data and j_data["number"] != i_data["number"]:
            new_item = MemoryItem()
            new_item.update(j_data)
            self.__data.append(new_item)

    def compare_and_update(self, other_memory_data):
        for item in other_memory_data:
            self.check_and_update_memory_about_j(item)


