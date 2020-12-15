##############################################Help classes#########################################################

#class used in task3 during backtracking in order to reverse list in O(n) times
#where n is number of items in stack
class Stack:
    def __init__(self):

        self.count = 0
        self.top = -1
        self.array = []

    def is_full(self):
        return self.count >= len(self.array)

    def is_empty(self):
        return self.count == 0

    def push(self, item):
        self.top += 1
        self.array.append(item)
        self.count += 1

    def pop(self):
        if self.is_empty():
            raise Exception("stack is empty")
        tobereturn = self.array[self.top]
        self.top -= 1
        self.count -= 1
        return tobereturn

    def __len__(self):
        return self.count

#class node needed for class queue for task 2
class Node:
    def __init__(self,item,next):
        self.item = item
        self.next = next

#class that represents linked queue for task 2
class Queue:
    def __init__(self):
        self.rear = None
        self.front = None
        self.count = 0

    def is_empty(self):
        return self.count == 0

    def append(self, item):
        new_node = Node(item, None)

        if self.is_empty():
            self.front = new_node
        else:
            self.rear.next = new_node
        self.rear = new_node

        self.count+=1

    # function to perform pop action (get rid of item in front)
    # takes O(1) time
    def serve(self):
        if self.is_empty():
            raise IndexError("queue is empty")
        item = self.front.item
        self.front = self.front.next
        if self.is_empty():
            self.rear = None
        self.count -=1
        return item

#Min heap used to optimize Dijkstra
class Heap:
    def __init__(self,size= 100):
        self.count = 0
        self.array = [None] * size

    def is_empty(self):
        return self.count == 0

    def add(self, item):
        if self.count + 1 < len(self.array):
            self.array[self.count + 1] = item
        else:
            raise Exception("Heap is full")
        self.count += 1
        self.rise(self.count)


    def swap(self, i, j):
        self.array[i], self.array[j] = self.array[j], self.array[i]

    def rise(self, node):
        while node > 1 and self.array[node][1] < self.array[node // 2][1]:
            self.swap(node // 2, node)
            node = node // 2


    def sink(self, node):
        while node * 2 <= self.count:
            child = self._get_smallest_child(node)
            if self.array[child][1] > self.array[node][1]:
                break
            self.swap(node, child)
            node = child


    def _get_smallest_child(self, node):
        if node * 2 == self.count or self.array[node * 2][1] < self.array[2 * node + 1][1]:
            return 2 * node
        else:
            return 2 * node + 1

    def update(self, vertex, new_distance):
        for i in range(1, len(self.array)):
            if self.array[i] == vertex:
                self.array[i][1] = new_distance

    def pop(self):
        retval = self.array[1]
        self.swap(1, self.count)
        self.count -= 1
        self.sink(1)

        return retval

#######################################################Assignment classes#################################################################

class Vertex:
    def __init__(self, id):
        self.id = id
        self.edges = []

    def add_edge(self, edge):
        self.edges.append(edge)

class Edge:
    def __init__(self, u, v, w = 0):
        self.u = u
        self.v = v
        self.w = w

class Graph:
    def __init__(self, gfile):
        self.vertices = []
        self.read_file(gfile)
        self.max_distances = []

    def get_vertex(self, vertex):
        return self.vertices[vertex]

    def add_edge(self,u,v, w):
        vertex_u = self.get_vertex(u)
        vertex_v = self.get_vertex(v)

        edge = Edge(vertex_u, vertex_v, w)
        vertex_u.add_edge(edge)

        #since the graph is undirected, meaning every edge goes both ways
        #therefore we add edge back to every given edge
        edge = Edge(vertex_v,vertex_u, w)
        vertex_v.add_edge(edge)

    def read_file(self, gfile):
        file = open(gfile, 'r')
        list = []
        for line in file:
            line = line.split()
            list.append(line)

        self.size = int(list[0][0])
        for i in range(self.size):
            v = Vertex(i)
            self.vertices.append(v)

        for i in range(1, len(list)):
            vertex_u = int(list[i][0])
            vertex_v = int(list[i][1])
            w = int(list[i][2])

            self.add_edge(vertex_u, vertex_v, w)


    def shallowest_spanning_tree(self):
        min_id = 0
        for i in range(self.size):
            self.bfs(i)
            if len(self.max_distances) > 1 and self.max_distances[i][1] < self.max_distances[min_id][1]:
                min_id = i


        return self.max_distances[min_id][0], self.max_distances[min_id][1]

    def bfs(self, s):
        s = self.get_vertex(s)
        distance = [-1] * self.size
        distance[s.id] = 0
        max_distance = 0
        queue = Queue()
        queue.append(s)

        while not queue.is_empty():
            u = queue.serve()
            for e in u.edges:
                if distance[e.v.id] == -1:
                    distance[e.v.id] = distance[e.u.id] + 1
                    if distance[e.u.id] + 1 > max_distance:
                        max_distance = distance[e.u.id] + 1

                    queue.append(e.v)

        self.max_distances.append((s.id, max_distance))


    def dijkstra(self, s):
        s = self.get_vertex(s)
        distance = [9999999] * self.size
        previous = [None] * self.size
        distance[s.id] = 0
        queue = Heap(self.size)
        queue.add([s, 0])
        while not queue.is_empty():
            u = queue.pop()
            key = u[1]
            u = u[0]
            if distance[u.id] <= key:
                for e in u.edges:
                    if distance[e.v.id] > distance[e.u.id] + e.w:
                        distance[e.v.id] = distance[u.id] + e.w
                        previous[e.v.id] = e.u.id
                        queue.add([e.v, distance[e.v.id]])


        return distance, previous


    def backtrack(self, previous, home, start, final_path, total_distance):
        p = Stack()
        while start != home:
            p.push(start)
            start = previous[start]

        while not p.is_empty():
            final_path.append(p.pop())
            total_distance +=1


        return final_path, total_distance


    def shortest_errands(self, home, destination, ice_locs, ice_cream_locs):
        distance, previous = self.dijkstra(home)
        total_distance = 0
        final_path = []

        final_path.append(home)

        d = distance[ice_locs[0]]
        ci = ice_locs[0]
        for i in ice_locs:
            if distance[i] < d:
                d = distance[i]
                ci = i
        final_path, total_distance = self.backtrack(previous, home, ci, final_path, total_distance)

        distance, previous = self.dijkstra(ci)

        d = distance[ice_cream_locs[0]]
        cic = ice_cream_locs[0]
        for i in ice_cream_locs:
            if distance[i] < d:
                d = distance[i]
                cic = i
        final_path, total_distance = self.backtrack(previous, ci, cic, final_path, total_distance)

        distance, previous = self.dijkstra(cic)
        final_path,total_distance = self.backtrack(previous, cic, destination, final_path, total_distance)

        return total_distance, final_path


z = Graph('test1')

print(z.dijkstra(1))