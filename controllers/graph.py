import numpy as np


class Graph:
    '''
    This class implements functionality for simple, undirected graphs.
    '''
    def __init__(self, adjacency_list):

        self._adjacency = self.get_undirected_graph(adjacency_list)
        self._matrix = self.list2matrix(self._adjacency)
        self.num_nodes = len(self._adjacency.keys())

        self._visited = []
        self._connected_components = []
        self._compute_connected_components = False
        self._priorities = [ k for k in range(self.num_nodes) ]

        self._sort_priorities = False
        self.swap_complete = True

    def get_undirected_graph(self, adjacency_list):
        '''
        This function returns the corresponding (undirected) graph and removes self loops.
        '''
        for node in adjacency_list.keys():
            for vertex in adjacency_list[node]:
                if node not in adjacency_list[vertex]:
                    adjacency_list[vertex].append(node)
                if node == vertex:
                    adjacency_list[node].remove(vertex)
        return adjacency_list

    def list2matrix(self, adjacency_list):
        '''
        Converts adjacency list to adjacency matrix.
        '''
        num_nodes = len(adjacency_list.keys())
        adjacency_matrix = np.zeros([num_nodes, num_nodes])
        for node in adjacency_list.keys():
            for vertex in adjacency_list[node]:
                adjacency_matrix[node, vertex] = 1

        adjacency_matrix += adjacency_matrix.T
        adjacency_matrix = np.clip( adjacency_matrix, a_min=0, a_max=1 )

        return adjacency_matrix

    def matrix2list(self, adjacency_matrix):
        '''
        Converts adjacency matrix to adjacency list.
        '''
        num_nodes = len(adjacency_matrix)
        adjacency_list = {}
        for i in range(num_nodes):
            adjacency_list[i] = []
            for j in range(i+1, num_nodes):
                if adjacency_matrix[i,j] != 0.0:
                    adjacency_list[i].append(j)
        
        return self.get_undirected_graph(adjacency_list)

    def get_adjacency_list(self):
        '''
        Returns corresponding adjacency list.
        '''
        return self._adjacency

    def set_adjacency_list(self, adjacency_list):
        '''
        Configures the graph adjacency list.
        '''
        self._adjacency = self.get_undirected_graph(adjacency_list)
        self._matrix = self.list2matrix(self._adjacency)

    def get_adjacency_matrix(self):
        '''
        Returns corresponding adjacency matrix.
        '''
        self._matrix = self.list2matrix(self._adjacency)
        return self._matrix

    def set_adjacency_matrix(self, adjacency_matrix):
        '''
        Configures the graph adjacency matrix.
        '''
        self._adjacency = self.matrix2list(adjacency_matrix)
        self._matrix = self.list2matrix(self._adjacency)

    def show(self):
        '''
        Prints the graph.
        '''
        print("Graph has " + str(len(self._adjacency.keys())) + " nodes.")
        for items in self._adjacency.items():
            print("Node " + str(items[0]) + " is connected to nodes " + str(items[1]) + ". Node " + str(items[0]) + " has priority " + str(self._priorities[items[0]]))

    def add_node(self, new_node):
        '''
        Adds new node to graph, with empty vertex list.
        '''
        if new_node in self._adjacency.keys():
            raise Exception("Node already exists.")
        else:
            self._adjacency[new_node] = []

    def remove_node(self, node_to_delete):
        '''
        Removes a node from graph.
        '''
        del self._adjacency[node_to_delete]
        for node in self._adjacency.keys():
            for connected_node in self._adjacency[node]:
                if connected_node == node_to_delete:
                    self._adjacency[node].remove(connected_node)

    def add_connection(self, node1, node2):
        '''
        Adds connection between node1 and node2.
        '''
        if node2 not in self._adjacency[node1]:
            self._adjacency[node1].append(node2)
        if node1 not in self._adjacency[node2]:
            self._adjacency[node2].append(node1)

    def remove_connection(self, node1, node2):
        '''
        Removes connection between node1 and node2, if it exists.
        '''
        if node1 in self._adjacency.keys() and node2 in self._adjacency.keys():
            if node2 in self._adjacency[node1]:
                self._adjacency[node1].remove(node2)
            if node1 in self._adjacency[node2]:
                self._adjacency[node2].remove(node1)
        else:
            raise Exception("Node does not exist.")

    def get_vertices(self, node):
        '''
        Gets the current vertices of a node.
        '''
        if node in self._adjacency.keys():
            return self._adjacency[node]
        else:
            raise Exception("Node does not exist.")

    def get_connected_components(self):
        '''
        Find connected components of a graph.
        '''
        self._visited = []
        self._connected_components = []

        self._compute_connected_components = True
        for node in self._adjacency.keys():
            self.dfs_iter(node)
        self._compute_connected_components = False

        return self._connected_components

    def get_priorities(self):
        '''
        Returns graph priorities.
        '''
        return self._priorities

    def set_priorities(self, priorities):
        '''
        Sets priorities for the graph.
        '''
        self._priorities = priorities

    def priority_sorting(self, desired_priorities):
        '''
        Executes priority sorting.
        '''        
        connected_components = self.get_connected_components()

        # For each connected component, executes DFS starting from the max. priority node.
        self._sort_priorities = True
        for component in connected_components:

            component_priorities = [ desired_priorities[node] for node in component ]
            max_priority_node = np.argmax(component_priorities)

            self.swap_complete = False
            while not self.swap_complete:
                self._visited = []
                self.swap_complete = True
                self.dfs_iter(max_priority_node)
        self._sort_priorities = False
        
        return self._priorities

    def dfs_iter(self, node):
        '''
        Basic recursive iteration from the DFS algorithm.
        '''
        if node not in self._visited:
            self.dfs_on_enter_unvisited_node(node)
            self._visited.append(node)
            for neighbour_node in self._adjacency[node]:
                self.dfs_on_enter_neighbor(node, neighbour_node)
                self.dfs_iter(neighbour_node)
        else:
            self.dfs_on_enter_visited_node(node)

    def dfs_on_enter_unvisited_node(self, node):
        '''
        Executes immediately after DFS enters an unvisited node.
        '''
        if self._compute_connected_components:
            new_component = True
            for neighbour_node in self._adjacency[node]:
                if neighbour_node in self._visited:
                    new_component = False
                    break
            if new_component:
                self._connected_components.append( set() )
            self._connected_components[-1].add(node)

    def dfs_on_enter_neighbor(self, node, neighbour_node):
        '''
        Executes immediately after DFS enters the neighbour of a node.
        '''
        if self._sort_priorities:
            if neighbour_node not in self._visited:
                if self._priorities[neighbour_node] > self._priorities[node]:
                    self._priorities[neighbour_node], self._priorities[node] = self._priorities[node], self._priorities[neighbour_node] # swap priorities
                    self.swap_complete = False

    def dfs_on_enter_visited_node(self, node):
        '''
        Executes immediately after DFS enters an already visited node.
        '''
        pass