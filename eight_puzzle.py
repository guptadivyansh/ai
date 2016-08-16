# Script to benchmark different algos for solving 8-puzzle (or n-puzzle)
#
# Author: Divyansh Gupta (divyanshgupta95@gmail.com)
#
# Basic idea is to represent board configurations as nodes in a graph, 
# with edges to configurations that can be reached by one swap, 
# and then using a graph search algorithm for finding a path to the goal configuration.
#
# Currently, A* and IDA* have been implemented. A* works pretty well but IDA* is slow.
# However, if the state space is very large, A* might run into memory issues.
# The return value of the algos can be used to generate a step-by-step solution
# to any solvable initial board configuration.

import heapq
import random
import statistics
import time
import math

# Note: Only tested with 3x3 and 3x4 boards
board_height = 3
board_width = 3
blank_val = board_height * board_width - 1 # value 8 represents the blank tile in 8-puzzle

class Node:
	def __init__(self, state, h_func, g_value = 0):
		self.state = state
		self.h_func = h_func
		self.g_value = g_value
		self.f_value =  self.g_value + self.h_func(self.state)
		self.blank = find_blank(self.state)
		self.parent = None
		self.action = None

	def __hash__(self):
		'''Used for set and dict'''
		return hash(self.state)

	def __eq__(self, other):
		'''Used for set and dict'''
		return self.state == other.state

	def __ne__(self, other):
		return self.state == other.state

	def __lt__(self, other):
		'''Used for priority queue'''
		return self.f_value < other.f_value

	def __str__(self):
		for i in range(board_height):
			print("|", end = ' ')
			for j in range(board_width):
				if(self.state[i * board_width + j] == blank_val):
					print("X" + " | ", end = "")
				else:
					print(str(self.state[i*board_width + j] + 1) + " | ", end = "")
			print("\n")	

	def swapped(self, index):
		'''Returns a new node with blank tile swapped with tile at position index'''
		assert(abs(self.blank - index) == 1 or abs(self.blank - index) == board_width)
		i, j = sorted([index, self.blank])
		state = self.state
		new_state = state[:i] + (state[j],) + state[i+1:j] + (state[i],) + state[j+1:]
		new_node = Node(new_state, self.h_func, self.g_value + 1)
		new_node.parent = self
		return new_node

	def shuffled(self, swaps=20):
		"""Return a new position after making 'swaps' swaps."""
		result = self
		for _ in range(swaps):
			result = random.choice(list(result.neighbors()))
		return result


	def neighbors(self):
		blank_y, blank_x = divmod(self.blank, board_width)
		neighbor = []
		if blank_y > 0:
			new_node = self.swapped(self.blank - board_width)
			new_node.action = "down"
			neighbor.append(new_node)
		if blank_y < board_height - 1:
			new_node = self.swapped(self.blank + board_width)
			new_node.action = "up"
			neighbor.append(new_node)
		if blank_x > 0:
			new_node = self.swapped(self.blank - 1)
			new_node.action = "right"
			neighbor.append(new_node)
		if blank_x < board_width - 1:
			new_node = self.swapped(self.blank + 1)
			new_node.action = "left"
			neighbor.append(new_node)
		return neighbor


def h_a(state):
	return 0

def h_b(state):
	misplaced = 0
	for i in range(len(state)):
		if state[i] != i:
			misplaced += 1
	return misplaced - 1 #don't count blank tile

def h_c(state):
	distance = 0
	for i in range(len(state)):
		if state[i] != len(state) - 1:
			pos_y, pos_x = divmod(i, board_width)
			tile_y, tile_x = divmod(state[i], board_width)
			distance += abs(pos_y - tile_y) + abs(pos_x - tile_x)
	return distance


def find_blank(state):
	return state.index(blank_val)

def is_solvable(state):
	'''Not all initial board configurations are solvable'''
	parity = 0	
	for i  in range(len(state)):
		for j in range(i + 1, len(state)):
			if state[i] > state[j] and state[j] != blank_val and state[i] != blank_val:
				parity += 1
	if board_width % 2 == 1:
		return (parity % 2 == 0)
	else:
		blank_y, blank_x = divmod(find_blank(state), board_width)
		if (board_height - blank_y) % 2 == 1: 
			return (parity % 2 == 0)
		else:
			return (parity % 2 == 1)

def a_star(start, goal):
	'''Returns list of nodes in the shortest path between start and goal, and the number of nodes visited during execution.

	Note: The node class has __lt__ defined on f_value (used in priority_queue) and __eq__ defined on state (board configuration) (used in set/dict). Thus, nodea == nodeb and nodea < nodeb may 		both be true if they represent the same board configuration but have different path costs'''
	expanded_count = 0
	frontier = [start] # used as heapq for efficient priority queue
	frontier_set = {start : start} # dict used for O(1) set membership test
	explored = set() # set used for O(1) set membership test
	while frontier:
		expanded_count += 1
		current = heapq.heappop(frontier)
		frontier_set.pop(current)
		# print("\n\nCurrent: ")
		# current.__str__()
		# print("parent: ")
		# current.parent.__str__()
		if current == goal:
			solution = []
			while True:
				solution.append(current)
				if current.parent is None:
					break
				current = current.parent
			return expanded_count, list(reversed(solution))		
		else:
			explored.add(current)
			for neighbor in current.neighbors():
				if neighbor not in explored and neighbor not in frontier_set:
					heapq.heappush(frontier, neighbor)
					frontier_set[neighbor] = neighbor
				elif neighbor in frontier_set:
					if neighbor < frontier_set[neighbor]:
						frontier.remove(neighbor) # removes the neighbor node with old value of f(n)
						heapq.heapify(frontier)
						frontier_set.pop(neighbor)
						heapq.heappush(frontier, neighbor)
						frontier_set[neighbor] = neighbor
	return None


def ida_star(start, goal):
	'''Returns list of nodes in the shortest path between start and goal, and the number of nodes visited during execution'''
	bound = start.f_value
	expanded_count = 0
	while True:
		#print(">>Looking with bound = ", bound)
		t = search(start, bound, goal)
		if t[0] == "FOUND":
			return expanded_count + t[2], list(reversed(t[1]))
		if t[0] == math.inf:
			return None
		bound = t[1]
		expanded_count += t[2]

def search(node, bound, goal):
	'''Auxillary function for ida_star, behaves like dfs restricted by f(n) < bound. If the path has been found, the second return value returns the list of nodes in the path to goal, else it returns the smallest f(n) value which is just above bound. Also returns the number of nodes expanded.'''
	if node.f_value > bound:
		return "LOOKING", node.f_value, 1
	if node == goal:
		return "FOUND", [node], 1
	minimum = math.inf
	expanded_count = 1
	for neighbor in node.neighbors():
		t = search(neighbor, bound, goal)
		expanded_count += t[2]
		if t[0] == "FOUND":
			t[1].append(node)
			return t[0], t[1], expanded_count
		if t[1] < minimum:
			minimum = t[1]
	return "LOOKING", minimum, expanded_count

# Note: h_func of goal node doesn't matter as the __eq__ method only checks the board state
goal_node = Node(tuple(range(board_width * board_height)), h_b)
# start_node = goal_node.shuffled(1000)
# start_node.parent = None
# start_node.action = None

benchmark_iterations = 100

state_list = []
states_file = open("states.txt", "w")
for i in range(benchmark_iterations):	
	l = list(range(board_width * board_height))
	random.shuffle(l)
	#print(str(l))
	while not is_solvable(l):
		random.shuffle(l)
		#print(str(l))
	states_file.write(str(l) + "\n")
	state_list.append(tuple(l))
states_file.close()

stats_file = open("stats.txt", "w")
stats_file.write("algo h_function average_time minimum_expanded maximum_expanded average_expanded stdev_expanded\n")
stats_file.close()
for algo in [a_star, ida_star]:
	for h_func in [h_c, h_b, h_c]:
		print("Running " + algo.__name__ + " with heuristic " + h_func.__name__)
		expanded_counts = []
		sol_lens = []
			
		start_time = time.clock()
		for index, start_state in enumerate(state_list):
			start_node = Node(start_state, h_func)

			#print("Start:")
			#start_node.__str__()

			expanded_count, solution = algo(start_node, goal_node)
			expanded_counts.append(expanded_count)
			sol_lens.append(len(solution))

			print("State: " + str(index) + " Expanded: " + str(expanded_count) + " Solution length: " + str(len(solution)))
			print(">>>Length of solution = ", len(solution), "Number of nodes expanded = ", expanded_count)
			
			for node in solution:
				if node.action:
					print("Move: ", node.action)
				node.__str__()
		end_time = time.clock()
		average_time = (end_time - start_time) / benchmark_iterations
		minimum = min(expanded_counts)
		maximum = max(expanded_counts)
		average = statistics.mean(expanded_counts)
		stddev = statistics.pstdev(expanded_counts)
		stats_file = open("stats.txt", "a")
		stats_file.write(algo.__name__ + " " + h_func.__name__ + " " + str(average_time) + " " + str(minimum) + " " + str(maximum) + " " + str(average) + " " + str(stddev) + "\n")
		stats_file.close()
		expanded_file = open(algo.__name__ + "_" + h_func.__name__ + ".txt", "w")
		expanded_file.write("solution_length nodes_expanded\n")
		for length, expanded in zip(sol_lens, expanded_counts):
			expanded_file.write(str(length) + " " + str(expanded) + "\n")
		expanded_file.close()


