import search
import random
import math
import utils
import itertools
import pickle
import copy

ids = ["208608802", "318514395"]


class DroneProblem(search.Problem):
    """This class implements a medical problem according to problem description file"""

    @staticmethod
    def get_needed_packages(clients: dict) -> list:
        result = {}
        for client, client_data in clients.items():
            for i in range(len(client_data['packages'])):
                result[client_data['packages'][i]]=client
        return result

    @staticmethod
    def get_manhattan_distance(source, destination):
        return abs(source[0] - destination[0]) + abs(source[1] - destination[1])

    def __init__(self, initial):
        """Don't forget to implement the goal test
        You should change the initial to your own representation.
        search.Problem.__init__(self, initial) creates the root node"""
        self.map = initial["map"]
        self.packages_locations = initial["packages"]
        # ignore packages that are not required by any client, because we don't need to deliver
        self.needed_packages = self.get_needed_packages(initial["clients"])
        self.clients = initial["clients"]
        for drone, pos in initial["drones"].items():
            initial["drones"][drone] = [pos, []]
        #remaining_packages = list(filter(lambda package: package in self.packages_locations.keys(), self.needed_packages))
        remaining_packages=list(self.needed_packages.keys())
        client_index_in_path_dict = dict([(client_name, 0) for client_name in self.clients])
        initial = [initial["drones"], remaining_packages, [], client_index_in_path_dict]
        search.Problem.__init__(self, pickle.dumps(initial))

    def actions(self, state):
        """Returns all the actions that can be executed in the given
        state. The result should be a tuple (or other iterable) of actions
        as defined in the problem description file"""

        state = pickle.loads(state)
        clients_index_in_path_dict = state[3]
        drones_locations_and_current_packages_list_dict = state[0]
        remaining_packages = state[1]
        all_drone_actions = []
        clients_index_in_path_dict=state[3]
        for drone_name, location_and_current_packages in drones_locations_and_current_packages_list_dict.items():
            drone_location = location_and_current_packages[0]
            this_drone_actions = []

            # check where the drone can move
            # right
            if drone_location[1] + 1 < len(self.map[0]) and self.map[drone_location[0]][drone_location[1] + 1] == 'P':
                this_drone_actions.append(("move", drone_name, (drone_location[0], drone_location[1] + 1)))
            # left
            if drone_location[1] > 0 and self.map[drone_location[0]][drone_location[1] - 1] == 'P':
                this_drone_actions.append(("move", drone_name, (drone_location[0], drone_location[1] - 1)))
            # down
            if drone_location[0] + 1 < len(self.map) and self.map[drone_location[0] + 1][drone_location[1]] == 'P':
                this_drone_actions.append(("move", drone_name, (drone_location[0] + 1, drone_location[1])))
            # up
            if drone_location[0] > 0 and self.map[drone_location[0] - 1][drone_location[1]] == 'P':
                this_drone_actions.append(("move", drone_name, (drone_location[0] - 1, drone_location[1])))

            drone_packages = location_and_current_packages[1]
            # check if the drone can pick up a package

            if len(drone_packages) < 2:
                for package in remaining_packages:
                    if self.packages_locations[package][0] == drone_location[0] \
                            and self.packages_locations[package][1] == drone_location[1]:
                        this_drone_actions.append(("pick up", drone_name, package))

            # check if the drone can drop package
            if len(drone_packages)!=0:
                for i in range (len(drone_packages)):
                        the_package=drone_packages[i]
                        client_that_want_our_package=self.needed_packages[the_package]
                        client_path = self.clients[client_that_want_our_package]["path"]
                        client_index_in_path = clients_index_in_path_dict[client_that_want_our_package]
                        client_location = client_path[client_index_in_path]
                        if client_location[0] == drone_location[0] and client_location[1] == drone_location[1]:
                            this_drone_actions.append(("deliver", drone_name, client_that_want_our_package,the_package))

            this_drone_actions.append(("wait", drone_name))

            all_drone_actions.append(this_drone_actions)

        # merge the drones actions
        # need to remove actions where two different drones pick same package
        merged_actions = list(itertools.product(*all_drone_actions))
        merged_actions = list(filter(self.is_legal_action, merged_actions))
        #merged_actions = list(tuple(action) for action in merged_actions)
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

    def result(self, state, action):
        """Return the state that results from executing the given
        action in the given state. The action must be one of
        self.actions(state)."""
        state = pickle.loads(state)
        clients_index_in_path_dict = state[3]
        drones_locations_and_current_list_dict = state[0]
        remaining_packages = state[1]
        dropped_packages = state[2]

        # apply action for all drones
        for drone_action in action:
            if drone_action[0] == "move":
                drone_to_move = drone_action[1]
                drone_new_location = drone_action[2]
                drones_locations_and_current_list_dict[drone_to_move][0] = drone_new_location
            if drone_action[0] == "pick up":
                drone_that_picks = drone_action[1]
                package_to_pick = drone_action[2]
                remaining_packages.remove(package_to_pick)
                drones_locations_and_current_list_dict[drone_that_picks][1].append(package_to_pick)
            if drone_action[0] == "deliver":
                drone_that_delivers = drone_action[1]
                package_to_deliver = drone_action[3]
                drones_locations_and_current_list_dict[drone_that_delivers][1].remove(package_to_deliver)
                dropped_packages.append(package_to_deliver)
                # add the package to the delivered packages after it is added
            if drone_action[0] == "wait":
                pass

        # move the clients
        for client_name in self.clients:
            client_path_len = len(self.clients[client_name]["path"])
            clients_index_in_path_dict[client_name] += 1
            clients_index_in_path_dict[client_name] = clients_index_in_path_dict[client_name] % client_path_len
        state = [drones_locations_and_current_list_dict, remaining_packages, dropped_packages, clients_index_in_path_dict]
        return pickle.dumps(state)

    def goal_test(self, state):
        """ Given a state, checks if this is the goal state.
         Returns True if it is, False otherwise."""
        state = pickle.loads(state)
        dropped_packages = state[2]
        if len(self.needed_packages) == len(dropped_packages):
            return True
        return False

    def h(self, node):
        """ This is the heuristic. It gets a node (not a state,
        state can be accessed via node.state)
        and returns a goal distance estimate"""

        if self.goal_test(node.state):
            return 0
        state = pickle.loads(node.state)
        remaining_package = state[1]
        remaining_packages=copy.copy(remaining_package)
        drone_locations_and_current_packages_dict = state[0]
        all_h = []
        score=0
        for drone_name, location_and_current_packages in drone_locations_and_current_packages_dict.items():
            num_of_packages = len(location_and_current_packages[1])
            if num_of_packages == 2:
                 dis,pak=self.get_distance_from_closest_client(location_and_current_packages[0], location_and_current_packages[1], state)
                 all_h.append(dis)
                 score+=1


            # can't drop packages, try to get closer to the closest package
            if num_of_packages == 0:
                if len(remaining_packages)>0:
                    dis,pak=self.get_distance_from_closest_package_for_2(location_and_current_packages[0], remaining_packages, state)
                    all_h.append(dis)
                    remaining_packages.remove(pak)
                else:
                    all_h.append(0)
            if num_of_packages == 1:
               if len(remaining_packages)>0:
                   dis,pak= self.get_distance_from_closest_package_for_2(location_and_current_packages[0], remaining_packages,state)
                   dis2,pak2= self.get_distance_from_closest_client(location_and_current_packages[0], location_and_current_packages[1], state)
                   all_h.append(min(dis,dis2))
                   if dis < dis2:
                       remaining_packages.remove(pak)
               else:
                   dis2, pak2 = self.get_distance_from_closest_client(location_and_current_packages[0],
                                                                      location_and_current_packages[1], state)
                   all_h.append(dis2)




        dropped_packages = state[2]
        all_h = list(filter(lambda x: x is not None, all_h))
        if not all_h:
            # check for cases that all_h didn't have values that are not None, can happen in unsolvable problems
            all_h.append(1)
        max_dist = max(all_h)
        avg_dist = sum(all_h) / len(all_h)+ 1
        res = node.depth + len(set(self.needed_packages) - set(dropped_packages)) * avg_dist + len(
            remaining_packages) * avg_dist + max_dist

        return res


    def get_distance_from_closest_client(self, location, current_packages,state):
        clients_index_in_path_dict = state[3]
        distances_from_clients = []
        for i in range(len(current_packages)):
                clients_that_want_one_of_our_packages = self.needed_packages[current_packages[i]]
                client_path = self.clients[clients_that_want_one_of_our_packages]["path"]
                client_path_len=len(client_path)
                client_index_in_path = clients_index_in_path_dict[clients_that_want_one_of_our_packages]+1
                client_location = client_path[(client_index_in_path) % client_path_len]
                distances_from_client = self.get_manhattan_distance(location,client_location)
                distances_from_clients.append(distances_from_client)
        return min(distances_from_clients),0

    def get_distance_from_closest_package_for_2(self, location, packages,state):
        if not packages:
            return 0,0


        min=1000000000
        for package in packages:
            distances_from_packages=self.get_manhattan_distance(location, self.packages_locations[package])
            client_that_want_one_of_our_packages = self.needed_packages[package]
            client_path = self.clients[client_that_want_one_of_our_packages]["path"]
            client_path_len = len(client_path)
            client_index_in_path = state[3][client_that_want_one_of_our_packages]
            client_location = client_path[(client_index_in_path) % client_path_len]
            dist_from_client=self.get_manhattan_distance(self.packages_locations[package], client_location)
            dist = distances_from_packages+dist_from_client
            if dist < min:
                min=dist
                package_deliverd=package



        return min,package_deliverd
    """Feel free to add your own functions
    (-2, -2, None) means there was a timeout"""


def create_drone_problem(game):
    return DroneProblem(game)
