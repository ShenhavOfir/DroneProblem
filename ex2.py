import itertools
import numpy as np
import copy
import random
import utils
from itertools import product

ids = ["111111111", "222222222"]


class DroneAgent:
    def __init__(self, initial):
        self.map = initial['map']
        self.initial_num_of_packages = len(initial["packages"])
        self.turns_from_reset = 0
        self.visited_states = []
        self.action_for_visited_state={}
        self.needed_packages = {}
        for client, client_data in initial["clients"].items():
            for i in range(len(client_data['packages'])):
                self.needed_packages[client_data['packages'][i]] = client
        self.height, self.width = len(self.map), len(self.map[0])
        self.distances = self.distance()
        self.times_of_pickups= {}

    def act(self, state):
        #state_old=(tuple(state["drones"]),tuple(state["clients"]),tuple(state["packages"]))
        #if state_old in self.action_for_visited_state:
        #       return self.action_for_visited_state[state_old]
        turns_left = state["turns to go"]
        if len(state["packages"]) == 0:
            if turns_left > self.turns_from_reset and self.initial_num_of_packages > 1:
                self.turns_from_reset = 0
                self.visited_states = []
                self.times_of_pickups = {}
                return "reset"
            else:
                return "terminate"
        self.turns_from_reset += 1
        actions = self.get_all_actions(state)
        best_value = np.inf
        best_action = None
        all_values = []
        for action in actions:
            if len(actions)>100:
                return random.choice(actions)
            new_state = self.apply_action(state, action)
            variant_state = (new_state["drones"], new_state["clients"], new_state["packages"])
            if variant_state in self.visited_states:
                continue
            value = self.calculate_value(new_state)
            all_values.append((value, action))
            if value < best_value:
                best_action = action
                best_value = value
        if best_action is None:
            best_action = random.choice(actions)
        #self.action_for_visited_state[state_old]=best_action
        print(best_action)
        return best_action


    # TODO: get all possible states(s') according to probabilities.
    #  then calculate for each s' its value using recursion or dynamic programming.
    def calculate_value(self, new_state):
        all_possible_states_and_their_probabilities = self.get_all_possible_states_and_their_probabilities(new_state)
        value = 0
        for state_and_probabilty in all_possible_states_and_their_probabilities:

            value += self.get_reward(state_and_probabilty[0]) * state_and_probabilty[1]

        return value

    def get_reward(self, state):
        turns_left = state["turns to go"]
        map_size = len(state["map"]) * len(state["map"][0])
        remaining_packages = dict([(package_name, state["packages"][package_name]) for package_name in state["packages"]
                                   if isinstance(state["packages"][package_name], tuple)])
        num_picked_packages = len([(package_name, state["packages"][package_name]) for package_name in state["packages"]
                                   if not isinstance(state["packages"][package_name], tuple)])
        num_deliverd_packages = self.initial_num_of_packages - num_picked_packages - len(remaining_packages)
        not_pick=self.initial_num_of_packages - num_picked_packages
        drone_locations_dict = state["drones"]
        all_h = []
        weight = 400 / len(state['drones'].keys()) ** 5
        score = 1
        for drone_name in drone_locations_dict:
            current_packages = self.get_current_packages(state, drone_name)
            current_location = drone_locations_dict[drone_name]
            drone_h ,score = self.get_h_for_drone(drone_name, current_location,current_packages, remaining_packages, state,score,not_pick)
            all_h.append(drone_h)

        max_dist = max(all_h) + 1
        avg_dist = sum(all_h) / len(all_h) + 1


        res = score+(self.initial_num_of_packages - num_deliverd_packages) *map_size*5+not_pick*map_size*3+avg_dist**2
        return res

    def get_current_packages(self, state, drone_name):
        packages = []
        for package in state["packages"]:
            if state["packages"][package] == drone_name:
                packages.append(package)
        return packages

    def get_h_for_drone(self, drone_name, location, current_packages, remaining_packages, state,score,not_pick):
        num_of_packages = len(current_packages)
        # can't pick any new packages, try to get closer to the one of the positions of a client that wants one of out package
        if num_of_packages == 2:

                return self.get_distance_from_closest_client(location, current_packages, state,score)
        # can't drop packages, try to get closer to the closest package
        if num_of_packages == 0:
            if len(remaining_packages.keys()) > 0:
                return self.get_distance_from_closest_package(location,remaining_packages, state),score*( len(remaining_packages)+1)
            else:
                return 0,score
        if num_of_packages == 1:
            if len(remaining_packages.keys()) > 0:
                dis, score1 = self.get_distance_from_2_closest_package(location,remaining_packages,state)
                dis2, score2 = self.get_distance_from_closest_client(location, current_packages, state,score)
                return min(dis,dis2), max(score1+len(remaining_packages),score2)
            else:
                return self.get_distance_from_closest_client(location, current_packages, state,score)

    def get_distance_from_closest_client(self, location, current_packages, state,score):
        clients_that_want_one_of_our_packages = \
            [client_name for client_name in state["clients"].keys() if
             set(state["clients"][client_name]["packages"]).intersection(current_packages)]
        distances_from_clients = []
        for client_name in clients_that_want_one_of_our_packages:
            distance_from_client = \
                self.distances[location, state["clients"][client_name]["location"]]
            distances_from_clients.append(distance_from_client)
        t1=self.times_of_pickups[current_packages[0]]
        t2=0
        if len(current_packages)>1:
            t2=self.times_of_pickups[current_packages[1]]

        return min(distances_from_clients),score+(t1+t2)

    def get_distance_from_closest_package(self, location, packages, state):
        if not packages:
            return 0
        distances_from_packages = [self.distances[location, packages[package]] for package
                                   in packages]
        return min(distances_from_packages)

    def get_distance_from_2_closest_package(self,current_location, remaining_packages_list,state):

        min=1000000000

        for package in remaining_packages_list:
            distances_from_packages=self.distances[current_location, remaining_packages_list[package]]
            client_that_want_one_of_our_packages = self.needed_packages[package]
            client_path = state["clients"][client_that_want_one_of_our_packages]["location"]
            distance_from_client = \
                self.distances[current_location, client_path]
            dist = distances_from_packages+distance_from_client
            if dist < min:
                min=dist

        return min,0

    def get_all_possible_states_and_their_probabilities(self, state):
        clients_data = state["clients"]
        world_map = state["map"]
        probabilities_for_all = []
        for client_name in clients_data:
            probabilities_for_client = list(clients_data[client_name]["probabilities"])
            client_location = clients_data[client_name]["location"]
            if client_location[0] == 0:
                probabilities_for_client[0] = 0
                self.recalculate_probabilities(probabilities_for_client)
            if client_location[0] == len(world_map) - 1:
                probabilities_for_client[1] = 0
                self.recalculate_probabilities(probabilities_for_client)
            if client_location[1] == 0:
                probabilities_for_client[2] = 0
                self.recalculate_probabilities(probabilities_for_client)
            if client_location[1] == len(world_map[0]) - 1:
                probabilities_for_client[3] = 0
                self.recalculate_probabilities(probabilities_for_client)

            probabilities_for_client[0] = (probabilities_for_client[0], client_name, (client_location[0] - 1, client_location[1]))
            probabilities_for_client[1] = (probabilities_for_client[1], client_name, (client_location[0] + 1, client_location[1]))
            probabilities_for_client[2] = (probabilities_for_client[2], client_name, (client_location[0], client_location[1] - 1))
            probabilities_for_client[3] = (probabilities_for_client[3], client_name, (client_location[0], client_location[1] + 1))
            probabilities_for_client[4] = (probabilities_for_client[4], client_name, (client_location[0], client_location[1]))

            probabilities_for_all.append(probabilities_for_client)

        merged_probabilities = list(itertools.product(*probabilities_for_all))

        new_states = []
        for probability_data in merged_probabilities:
            new_state = copy.deepcopy(state)
            only_probabilities = [x[0] for x in probability_data]
            total_probability = utils.product(only_probabilities)
            if total_probability > 0:

                for atomic_probability in probability_data:
                    new_state["clients"][atomic_probability[1]]["location"] = atomic_probability[2]
            new_states.append((new_state, total_probability))

        return new_states
    def distance(self):
        """
        Calculate the shortest distance between every pair of cells
        """
        distances = {}
        all_points = list(product(range(self.height), range(self.width)))
        infinity = float("inf")
        # Initialize the distances
        for point_1 in all_points:
            for point_2 in all_points:
                value = 0 if point_1 == point_2 else infinity
                distances[point_1, point_2] = value

        # Calculate the shortest paths
        for point_1 in all_points:
            for point_2 in all_points:
                for point_3 in all_points:
                    if self.neighbors(point_2, point_3) == 1:  # Neighbors
                        if self.map[point_3[0]][point_3[1]] != 'P':
                            weight = infinity
                        else:
                            weight = 1
                        distances[point_2, point_3] = weight
                    else:
                        distances[point_2, point_3] = min(distances[point_2, point_3],
                                                          distances[point_2, point_1] + distances[point_1, point_3])
        return distances

    def neighbors(self,point1,point2):
        if abs(point2[0]-point1[0])==1:
            if abs(point2[1]-point1[1])==1 or abs(point2[1]-point1[1])==0:
                return 1
        if abs(point2[0]-point1[0])==0:
            if abs(point2[1]-point1[1])==1:
                return 1
        return 0

    def recalculate_probabilities(self, probabilities):

        sum_of_probabilities = sum(probabilities)
        for i in range(len(probabilities)):
            probabilities[i] = probabilities[i] / sum_of_probabilities

    """
    create copy of state, with action applied
    """
    def apply_action(self, state, action):
        new_state = copy.deepcopy(state)
        for atomic_action in action:
            self.apply_atomic_action(new_state, atomic_action)
        return new_state


    def apply_atomic_action(self, state, atomic_action):
        action_name = atomic_action[0]
        drone_name = atomic_action[1]
        current_packages = self.get_current_packages(state, drone_name)
        if action_name == "wait":
            if len(current_packages)>0:
                for pak in current_packages:
                     self.times_of_pickups[pak] +=1
            return

        if action_name == "pick up":
            package = atomic_action[2]
            state["packages"][package] = drone_name
            self.times_of_pickups[package]=1

        if action_name == "move":
            destination = atomic_action[2]
            state["drones"][drone_name] = destination
            if len(current_packages)>0:
                for pak in current_packages:
                     self.times_of_pickups[pak] +=1


        if action_name == "deliver":
            package = atomic_action[3]
            state["packages"].pop(package)
            self.times_of_pickups[package] = 0

    def get_all_actions(self, state):
        world_map = state["map"]
        drones_location_by_name = state["drones"]
        remaining_packages = state["packages"]
        clients_data = state["clients"]
        all_drone_actions = []

        for drone_name in drones_location_by_name:
            drone_location = drones_location_by_name[drone_name]
            this_drone_actions = []

            # check where the drone can move
            # right
            if drone_location[1] + 1 < len(world_map[0]) and world_map[drone_location[0]][drone_location[1] + 1] == 'P':
                this_drone_actions.append(("move", drone_name, (drone_location[0], drone_location[1] + 1)))
            # left
            if drone_location[1] > 0 and world_map[drone_location[0]][drone_location[1] - 1] == 'P':
                this_drone_actions.append(("move", drone_name, (drone_location[0], drone_location[1] - 1)))
            # down
            if drone_location[0] + 1 < len(world_map) and world_map[drone_location[0] + 1][drone_location[1]] == 'P':
                this_drone_actions.append(("move", drone_name, (drone_location[0] + 1, drone_location[1])))
            # up
            if drone_location[0] > 0 and world_map[drone_location[0] - 1][drone_location[1]] == 'P':
                this_drone_actions.append(("move", drone_name, (drone_location[0] - 1, drone_location[1])))

            #right + down
            if drone_location[0] + 1 < len(world_map) and \
                    drone_location[1] + 1 < len(world_map[0]) \
                    and world_map[drone_location[0] + 1][drone_location[1] + 1] == 'P':
                this_drone_actions.append(("move", drone_name, (drone_location[0] + 1, drone_location[1] + 1)))

            #right + up
            if drone_location[0] > 0 and \
                    drone_location[1] + 1 < len(world_map[0]) \
                    and world_map[drone_location[0] - 1][drone_location[1] + 1] == 'P':
                this_drone_actions.append(("move", drone_name, (drone_location[0] - 1, drone_location[1] + 1)))

            #left + down
            if drone_location[0] + 1 < len(world_map) and \
                    drone_location[1] > 0 \
                    and world_map[drone_location[0] + 1][drone_location[1] - 1] == 'P':
                this_drone_actions.append(("move", drone_name, (drone_location[0] + 1, drone_location[1] - 1)))


            #left + up
            if drone_location[0] > 0 and \
                    drone_location[1] > 0 \
                    and world_map[drone_location[0] - 1][drone_location[1] - 1] == 'P':
                this_drone_actions.append(("move", drone_name, (drone_location[0] - 1, drone_location[1] - 1)))


            drone_packages = self.get_current_packages(state, drone_name)
            # check if the drone can pick up a package

            if len(drone_packages) < 2:
                for package, package_location in remaining_packages.items():
                    if not isinstance(state["packages"][package], tuple):
                        continue
                    if package_location[0] == drone_location[0] \
                            and package_location[1] == drone_location[1]:
                        this_drone_actions.append(("pick up", drone_name, package))

            # check if the drone can drop package
            for client_name in clients_data:
                for i in range(len(drone_packages)):
                    client_location = clients_data[client_name]["location"]
                    client_packages = clients_data[client_name]["packages"]
                    if drone_packages[i] in client_packages and client_location[0] == drone_location[0] \
                            and client_location[1] == drone_location[1]:
                        this_drone_actions.append(("deliver", drone_name, client_name, drone_packages[i]))

            this_drone_actions.append(("wait", drone_name))

            all_drone_actions.append(this_drone_actions)

        # merge the drones actions
        # need to remove actions where two different drones pick same package
        merged_actions = list(itertools.product(*all_drone_actions))
        merged_actions = list(filter(self.is_legal_action, merged_actions))
        merged_actions = list(tuple(action) for action in merged_actions)

        return merged_actions

    """
    verifies that the action is legal
    this means that 2 drones can't pick up the same package
    """

    @staticmethod
    def is_legal_action(action):
        for i in range(len(action)):
            if action[i][0] == "pick up":
                for j in range(i + 1, len(action)):
                    if action[j][0] == "pick up" and action[i][2] == action[j][2]:
                        return False
        return True

    @staticmethod
    def get_manhattan_distance(source, destination):
        return abs(source[0] - destination[0]) + abs(source[1] - destination[1])
