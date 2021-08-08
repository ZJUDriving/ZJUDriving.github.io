import sys
import random
import carla
# from carla import ColorConverter as cc
# import weakref
import numpy as np
from numpy.core.defchararray import not_equal
from utils import utils

class Basic_Env(object):
    def __init__(self, carla_world, args) -> None:
        super().__init__()
        self.world = carla_world
        try:
            self.map = self.world.get_map()
        except RuntimeError as error:
            print('RuntimeError: {}'.format(error))
            print('  The server could not send the OpenDRIVE (.xodr) file:')
            print('  Make sure it exists, has the same name of your town, and is correct.')
            sys.exit(1)
        
        self.player = None
        self.spectator = self.world.get_spectator()
        self.collision_sensor = None
        # self.camera = None
        # self.surface = None
        self.map = self.world.get_map()
        # self.waypoints = self.map.generate_waypoints(50)
        self.min_distance = args.aheaddist
        self.pre_time = args.pretime
        self.threshold_distance = self.min_distance

        self.distance_gap = args.d_step
        self.timestep = args.time_step

        self.blueprint = self.world.get_blueprint_library().find('vehicle.mercedes-benz.coupe')
        self.blueprint.set_attribute('color', '0, 0, 0')

        # self._camera_transforms = (carla.Transform(carla.Location(x=0.0, z=30.0), carla.Rotation(pitch=0.0)), carla.AttachmentType.SpringArm)
        # self.sensor = ['sensor.camera.rgb', cc.Raw, 'Camera RGB']
        # self._camera_blp = self.world.get_blueprint_library().find(self.sensor[0])
        # self._camera_blp.set_attribute('image_size_x', str(args.width))
        # self._camera_blp.set_attribute('image_size_y', str(args.height))
        
    def reset(self):

        print("Spawning the player")
        if self.player is not None:
            spawn_point = self.player.get_transform()
            spawn_point.location.z += 2.0
            spawn_point.rotation.roll = 0.0
            spawn_point.rotation.pitch = 0.0
            self.destroy()
            self.player = self.world.try_spawn_actor(self.blueprint, spawn_point)
        
        while self.player is None:
            if not self.map.get_spawn_points():
                print('There are no spawn points available in your map/town.')
                print('Please add some Vehicle Spawn Point to your UE4 scene.')
                sys.exit(1)
            spawn_points = self.map.get_spawn_points()
            spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
            self.player = self.world.try_spawn_actor(self.blueprint, spawn_point)
        
        self.spectator.set_transform(carla.Transform(self.player.get_transform().location + carla.Location(z=30),
                                                    carla.Rotation(pitch=-90)))

        # if self.camera is not None:
        #     self.sensor = None
        #     self.surface = None
        # self.camera = self.player.get_world().spawn_actor(
        #     self._camera_blp,
        #     self._camera_transforms[0],
        #     attach_to=self.player,
        #     attachment_type=self._camera_transforms[1]
        # )
        state = self.get_state()
        return state
        
    def render(self):
        self.spectator.set_transform(carla.Transform(self.player.get_transform().location + carla.Location(z=30),
                                                    carla.Rotation(pitch=-90)))
        # myself = self.map.get_waypoint(self.player.get_location())
        # state = self.get_state()
        # waypoints = state['waypoints']
        # for w in waypoints:
        #     for point in w:
        #         self.world.debug.draw_point(point.transform.location, life_time=120)
        # for w in self.waypoints:
        #     self.world.debug.draw_point(w.transform.location, size=0.1, life_time=0)
        # if self.surface is not None:
        #     self.display.blit(self.surface, (0, 0))
        # pygame.display.flip()
    
    def get_obstacle_around(self):
        actor_list = self.world.get_actors()
        vehicle_list = actor_list.filter("*vehicle*")

        my_pos = self.player.get_location()
        my_waypoint = self.map.get_waypoint(my_pos)

        obstacle_list = []
        direction = my_waypoint.transform.get_forward_vector()
        pre_distance = utils.dot(self.player.get_velocity(), direction) * self.pre_time
        
        self.threshold_distance = max(self.min_distance, pre_distance)

        for vehicle in vehicle_list:
            if vehicle.id == self.player.id:
                continue
            
            their_waypoint = self.map.get_waypoint(vehicle.get_location())
            if their_waypoint.road_id != my_waypoint.road_id:
                continue
                
            if abs(their_waypoint.s-my_waypoint.s) < self.threshold_distance:
                obstacle_list.append(vehicle)
        return obstacle_list  

    def step(self, action):    
        self.player.apply_control(action)
        state = self.get_state()
        return state

    def get_state(self):
        # obstacles = self.get_obstacle_around()
        obstacles = None
        my_pos = self.player.get_location()
        basepoint = self.map.get_waypoint(my_pos)
        waypoint_lists = []
        waypoint_lists.append(self.get_line_base(basepoint))
        nowpoint = basepoint
        index = 0
        while nowpoint.get_left_lane() != None and index < 1:
            nowpoint = nowpoint.get_left_lane()
            waypoint_lists.append(self.get_line_base(nowpoint))
            index += 1
        nowpoint = basepoint
        index = 0
        while nowpoint.get_right_lane() != None and index < 1:
            nowpoint = nowpoint.get_right_lane()
            waypoint_lists.append(self.get_line_base(nowpoint))
        
        return {'obstacles':obstacles, 'waypoints':waypoint_lists, 'mystate':self.player.get_transform(), 'myvelocity':self.player.get_velocity()}
    
    def get_line_base(self, basepoint):
        waypoint_list = []
        for i in np.arange(self.distance_gap, self.threshold_distance, self.distance_gap):
            waypoint_list.append(basepoint.next(i))
        return waypoint_list
    # def _is_light_red(self, lights_list):
    #     position = self.player.get_location()
    #     waypoint = self.map.get_waypoint(position)
        
    #     for traffic_light in lights_list:
    #         object_location = self._get_trafficlight_trigger_location(traffic_light)
    #         object_waypoint = self._map.get_waypoint(object_location)

    #         if object_waypoint.road_id != ego_vehicle_waypoint.road_id:
    #             continue
            
    #         ve_dir = ego_vehicle_waypoint.transform.get_forward_vector()
    #         wp_dir = object_waypoint.transform.get_forward_vector()
    #         dot_ve_wp = ve_dir.x * wp_dir.x + ve_dir.y * wp_dir.y + ve_dir.z * wp_dir.z

    #         if dot_ve_wp < 0:
    #             continue

    #         if is_within_distance_ahead(object_waypoint.transform,
    #                                     self._vehicle.get_transform(),
    #                                     self._proximity_tlight_threshold):
    #             if traffic_light.state == carla.TrafficLightState.Red:
    #                 return (True, traffic_light)

    #     return (False, None)

    def destroy(self):
        if self.player is not None:
            self.player.destroy()
