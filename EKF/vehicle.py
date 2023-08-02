
class Vehicle:
    def __init__(self, world, blueprint_library):
        self.world = world

        # Initialize the vehicle
        self.vehicle_name = 'vehicle.lincoln.mkz_2020'
        vehicle_bp = blueprint_library.find(self.vehicle_name)
        self.vehicle = world.spawn_actor(vehicle_bp, vehicle_transform)

    def set_autopilot_status(self, is_autopilot):
        self.vehicle.set_autopilot(is_autopilot)

    def destroy(self):
        self.vehicle.destroy()

    def get_location(self):
        return self.vehicle.get_location()    
        

