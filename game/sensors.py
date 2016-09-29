from panda3d.core import Vec3
from math import radians, cos, sin

from goap.sensors import SensorBase


class TargetLookingAtMe(SensorBase):

    def __init__(self, ai, threshold=0.95):
        self.threshold = threshold
        self.ai = ai

    def target_is_looking_at_me(self, agent, target):
        to_target = target.get_pos() - agent.get_pos()
        to_target.z = 0
        to_target.normalize()

        target_view_angle = target.get_hpr()[0]
        angle_rad = radians(target_view_angle)

        target_view_vector = Vec3(cos(angle_rad), sin(angle_rad), 0)
        target_view_vector.normalize()

        return target_view_vector.dot(to_target) > self.threshold

    def update(self):
        if self.ai.blackboard.target is None:
            return False

        if not self.ai.blackboard.target_is_visible:
            return False

        # TODO get confidence in visibilty

        is_looking = self.target_is_looking_at_me(self.ai.agent, self.ai.blackboard.target)

        return True


class TargetHasWeapon(SensorBase):

    def __init__(self, ai, threshold=0.95):
        self.threshold = threshold
        self.ai = ai

    def update(self):
        if self.ai.blackboard.target is None:
            return False

        if not self.ai.blackboard.target_is_visible:
            return False

        # TODO get confidence in wm

        return True