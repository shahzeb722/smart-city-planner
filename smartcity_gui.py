# smartcity_gui.py
# Run: python smartcity_gui.py
# Requires only standard library (tkinter, math, heapq, random, copy)

import tkinter as tk
from tkinter import simpledialog, messagebox
import math, heapq, random, copy, collections

# ---------------------------
# Data model: SmartCityState
# ---------------------------
class SmartCityState:
    def __init__(self):
        self.nodes = {}   # name -> (x,y)
        self.edges = {}   # name -> list of (neighbor, distance)
        self.tasks = {}   # task_name -> {'location': node, 'duration': hrs, 'desc': str}
        self.start_node = None
        self.goal_node = None

    def add_node(self, name, x, y):
        self.nodes[name] = (x, y)
        self.edges.setdefault(name, [])

    def remove_node(self, name):
        if name in self.nodes:
            del self.nodes[name]
        if name in self.edges:
            del self.edges[name]
        for n, nbrs in list(self.edges.items()):
            self.edges[n] = [(m,d) for (m,d) in nbrs if m != name]
        # remove tasks located at this node
        for t in list(self.tasks.keys()):
            if self.tasks[t]['location'] == name:
                del self.tasks[t]
        if self.start_node == name: self.start_node = None
        if self.goal_node == name: self.goal_node = None

    def add_edge(self, a, b, dist):
        self.edges.setdefault(a, [])
        self.edges.setdefault(b, [])
        # remove if already exists
        self.edges[a] = [(n,d) for (n,d) in self.edges[a] if n != b]
        self.edges[b] = [(n,d) for (n,d) in self.edges[b] if n != a]
        self.edges[a].append((b, dist))
        self.edges[b].append((a, dist))

    def set_task(self, task_name, location, duration, desc=""):
        self.tasks[task_name] = {'location': location, 'duration': duration, 'desc': desc}

    def euclidean(self, a, b):
        x1,y1 = self.nodes[a]; x2,y2 = self.nodes[b]
        return math.hypot(x2-x1, y2-y1)

    def neighbors(self, node):
        return self.edges.get(node, [])

    def copy(self):
        return copy.deepcopy(self)

# ---------------------------
# Search algorithms helpers
# ---------------------------
def a_star_shortest_distance(state, start, goal):
    """Return (path, cost) from start to goal using A* with Euclidean heuristic (on distances)."""
    if start not in state.nodes or goal not in state.nodes:
        return None, float('inf')
    pq = [(state.euclidean(start, goal), 0, start, [start])]  # (f, g, node, path)
    best_g = {start: 0}
    while pq:
        f, g, node, path = heapq.heappop(pq)
        if node == goal:
            return path, g
        if g != best_g.get(node, None):  # outdated entry
            continue
        for neigh, cost in state.neighbors(node):
            newg = g + cost
            if neigh not in best_g or newg < best_g[neigh]:
                best_g[neigh] = newg
                heapq.heappush(pq, (newg + state.euclidean(neigh, goal), newg, neigh, path + [neigh]))
    return None, float('inf')

def greedy_path(state, start, goal):
    """Greedy Best-First Search returning path (may not be optimal)."""
    if start not in state.nodes or goal not in state.nodes:
        return None
    pq = [(state.euclidean(start, goal), start, [start])]
    visited = set()
    while pq:
        h, node, path = heapq.heappop(pq)
        if node == goal:
            return path
        if node in visited:
            continue
        visited.add(node)
        for neigh, _ in state.neighbors(node):
            if neigh not in visited:
                heapq.heappush(pq, (state.euclidean(neigh, goal), neigh, path + [neigh]))
    return None

# ---------------------------
# CSP (Task 3): Backtracking
# ---------------------------
WORK_START = 9
WORK_END = 18

def make_csp_constraints_from_state(state):
    """
    Create constraint dict suitable for CSP solver.
    For demonstration we use common constraints:
    - T1 must finish before T3
    - T2 after T1
    - T5 cannot overlap T3
    These constraints assume tasks T1..T5 exist; otherwise adapt.
    """
    constraints = {}

    def end_time(task_var, assignment):
        return assignment[task_var] + state.tasks[task_var]['duration']

    # Only add constraints if tasks exist
    if 'T1' in state.tasks and 'T3' in state.tasks:
        constraints[('T1','T3')] = lambda a: end_time('T1', a) <= a['T3']
    if 'T1' in state.tasks and 'T2' in state.tasks:
        constraints[('T1','T2')] = lambda a: a['T2'] >= end_time('T1', a)
    if 'T3' in state.tasks and 'T5' in state.tasks:
        def c5(a):
            t5_ends = end_time('T5', a)
            t3_ends = end_time('T3', a)
            return (t5_ends <= a['T3']) or (a['T5'] >= t3_ends)
        constraints[('T3','T5')] = c5
    return constraints

class CSP:
    def __init__(self, variables, domains, constraints):
        self.variables = variables
        self.domains = domains
        self.constraints = constraints
        self.var_constraints = collections.defaultdict(list)
        for var_tuple, func in constraints.items():
            for v in var_tuple:
                self.var_constraints[v].append((var_tuple, func))

    def is_consistent(self, var, val, assignment):
        for var_tuple, func in self.var_constraints[var]:
            test = assignment.copy()
            test[var] = val
            if all(v in test for v in var_tuple):
                if not func(test):
                    return False
        return True

def csp_backtracking_solver(state):
    variables = list(state.tasks.keys())
    domains = {}
    for var in variables:
        dur = state.tasks[var]['duration']
        latest = WORK_END - dur
        domains[var] = list(range(WORK_START, latest+1))
    constraints = make_csp_constraints_from_state(state)
    csp = CSP(variables, domains, constraints)

    def backtrack(assignment):
        if len(assignment) == len(csp.variables):
            return assignment
        # MRV heuristic: choose var with smallest remaining domain
        unassigned = [v for v in csp.variables if v not in assignment]
        var = min(unassigned, key=lambda v: len(csp.domains[v]))
        for val in csp.domains[var]:
            if csp.is_consistent(var, val, assignment):
                assignment[var] = val
                res = backtrack(assignment)
                if res:
                    return res
                del assignment[var]
        return None

    return backtrack({})

# ---------------------------
# Optimization (Task 4): Simulated Annealing
# ---------------------------
HIGH_PENALTY = 1000

def schedule_fitness(state, schedule):
    """
    Fitness function:
    - base: span (max end - min start)
    - penalties for constraint violations (using CSP constraints)
    - travel time cost (assume agent visits tasks in order of start times)
    Lower is better.
    """
    if not schedule:
        return float('inf')
    starts = list(schedule.values())
    ends = [schedule[t] + state.tasks[t]['duration'] for t in schedule]
    span = max(ends) - min(starts)
    cost = span

    # constraint penalties
    constraints = make_csp_constraints_from_state(state)
    for key, func in constraints.items():
        try:
            if not func(schedule):
                cost += HIGH_PENALTY
        except Exception:
            cost += HIGH_PENALTY

    # travel time: order tasks by start time, sum shortest-path travel distances
    ordered = sorted(schedule.items(), key=lambda i: i[1])  # list of (task, start)
    travel = 0.0
    for i in range(len(ordered)-1):
        t1 = ordered[i][0]; t2 = ordered[i+1][0]
        loc1 = state.tasks[t1]['location']; loc2 = state.tasks[t2]['location']
        _, dcost = a_star_shortest_distance(state, loc1, loc2)
        if dcost == float('inf'):
            cost += HIGH_PENALTY  # unreachable
        else:
            travel += dcost
    cost += travel * 0.1  # weight travel modestly
    return cost

def get_random_neighbor_schedule(state, schedule):
    s = schedule.copy()
    t = random.choice(list(state.tasks.keys()))
    dur = state.tasks[t]['duration']
    latest = WORK_END - dur
    s[t] = random.randint(WORK_START, latest)
    return s

def simulated_annealing_optimize(state, initial_schedule, temp=1000.0, cooling=0.995, iterations=5000):
    current = initial_schedule.copy()
    current_cost = schedule_fitness(state, current)
    best = current.copy()
    best_cost = current_cost
    T = temp
    for i in range(iterations):
        if T <= 0.001: break
        neighbor = get_random_neighbor_schedule(state, current)
        neigh_cost = schedule_fitness(state, neighbor)
        diff = neigh_cost - current_cost
        if diff < 0 or random.random() < math.exp(-diff / T):
            current, current_cost = neighbor, neigh_cost
            if current_cost < best_cost:
                best, best_cost = current.copy(), current_cost
        T *= cooling
    return best, best_cost

# ---------------------------
# GUI: Graph + Controls
# ---------------------------
class SmartCityGUI:
    NODE_RADIUS = 16
    def __init__(self, root):
        self.root = root
        self.root.title("SmartCity Planner — Node/Route/CSP/Optimize")
        self.state = SmartCityState()

        # undo/redo stacks of deep copies
        self.undo_stack = []
        self.redo_stack = []

        # Canvas
        self.canvas = tk.Canvas(root, width=900, height=600, bg="white")
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Sidebar
        frame = tk.Frame(root)
        frame.pack(side=tk.RIGHT, fill=tk.Y)

        tk.Button(frame, text="Add Node (click)", command=self.set_mode_add_node).pack(fill=tk.X)
        tk.Button(frame, text="Add Node (enter coords)", command=self.add_node_via_input).pack(fill=tk.X)
        tk.Button(frame, text="Add Edge (select 2 nodes)", command=self.set_mode_add_edge).pack(fill=tk.X)
        tk.Button(frame, text="Edit Edge Distance", command=self.edit_edge_distance).pack(fill=tk.X)
        tk.Button(frame, text="Remove Node", command=self.set_mode_remove_node).pack(fill=tk.X)
        tk.Button(frame, text="Set Start Node", command=self.set_mode_set_start).pack(fill=tk.X)
        tk.Button(frame, text="Set Goal Node", command=self.set_mode_set_goal).pack(fill=tk.X)
        tk.Button(frame, text="Run A*", command=self.run_a_star).pack(fill=tk.X)
        tk.Button(frame, text="Run Greedy", command=self.run_greedy).pack(fill=tk.X)
        tk.Button(frame, text="Run CSP (schedule)", command=self.run_csp).pack(fill=tk.X)
        tk.Button(frame, text="Optimize (Simulated Annealing)", command=self.run_optimize).pack(fill=tk.X)
        tk.Button(frame, text="Undo", command=self.undo).pack(fill=tk.X)
        tk.Button(frame, text="Redo", command=self.redo).pack(fill=tk.X)
        tk.Button(frame, text="Clear All", command=self.clear_all).pack(fill=tk.X)

        # Info label
        self.info_var = tk.StringVar()
        tk.Label(frame, textvariable=self.info_var, wraplength=240, justify=tk.LEFT).pack(pady=10)
        self.set_info("Mode: add_node (click canvas)")

        # internal state
        self.mode = 'add_node'  # add_node, add_edge, remove_node, set_start, set_goal
        self.selected_node = None

        self.canvas.bind("<Button-1>", self.on_canvas_click)
        self.redraw()

        # Add sample nodes/edges/tasks for convenience
        self.add_sample_data()

    # ---------------------------
    # State snapshot helpers
    # ---------------------------
    def push_undo(self):
        self.undo_stack.append(self.state.copy())
        if len(self.undo_stack) > 50:
            self.undo_stack.pop(0)
        self.redo_stack.clear()

    def undo(self):
        if not self.undo_stack:
            messagebox.showinfo("Undo", "Nothing to undo")
            return
        self.redo_stack.append(self.state.copy())
        self.state = self.undo_stack.pop()
        self.selected_node = None
        self.redraw()

    def redo(self):
        if not self.redo_stack:
            messagebox.showinfo("Redo", "Nothing to redo")
            return
        self.undo_stack.append(self.state.copy())
        self.state = self.redo_stack.pop()
        self.selected_node = None
        self.redraw()

    # ---------------------------
    # Modes & UI actions
    # ---------------------------
    def set_info(self, text):
        self.info_var.set(text)

    def set_mode_add_node(self):
        self.mode = 'add_node'
        self.set_info("Mode: add_node. Click canvas to place a node.")

    def add_node_via_input(self):
        name = simpledialog.askstring("Node Name", "Enter node name (unique):")
        if not name: return
        try:
            x = int(simpledialog.askinteger("X coord", "Enter X coordinate (px)", minvalue=0, maxvalue=2000))
            y = int(simpledialog.askinteger("Y coord", "Enter Y coordinate (px)", minvalue=0, maxvalue=2000))
        except Exception:
            messagebox.showerror("Input", "Invalid coordinates.")
            return
        if name in self.state.nodes:
            messagebox.showerror("Name exists", "Node name already exists.")
            return
        self.push_undo()
        self.state.add_node(name, x, y)
        self.redraw()

    def set_mode_add_edge(self):
        self.mode = 'add_edge'
        self.selected_node = None
        self.set_info("Mode: add_edge. Click two nodes to create an edge (distance computed automatically).")

    def set_mode_remove_node(self):
        self.mode = 'remove_node'
        self.set_info("Mode: remove_node. Click a node to remove it and its edges.")

    def set_mode_set_start(self):
        self.mode = 'set_start'
        self.set_info("Mode: set_start. Click a node to mark start.")

    def set_mode_set_goal(self):
        self.mode = 'set_goal'
        self.set_info("Mode: set_goal. Click a node to mark goal.")

    def edit_edge_distance(self):
        # ask user to select two nodes by name and set distance
        a = simpledialog.askstring("Edge edit", "Enter first node name:")
        if not a or a not in self.state.nodes:
            messagebox.showerror("Error", "Invalid node")
            return
        b = simpledialog.askstring("Edge edit", "Enter second node name:")
        if not b or b not in self.state.nodes:
            messagebox.showerror("Error", "Invalid node")
            return
        found = False
        for neigh, d in self.state.edges.get(a, []):
            if neigh == b:
                found = True; break
        if not found:
            messagebox.showerror("Error", f"No edge between {a} and {b}")
            return
        newd = simpledialog.askfloat("Distance", f"Enter new distance between {a} and {b}:", minvalue=0.0)
        if newd is None:
            return
        self.push_undo()
        self.state.add_edge(a,b,float(newd))
        self.redraw()

    def clear_all(self):
        self.push_undo()
        self.state = SmartCityState()
        self.selected_node = None
        self.redraw()

    # ---------------------------
    # Canvas interactions
    # ---------------------------
    def on_canvas_click(self, event):
        x,y = event.x, event.y
        clicked = self.find_node_at_point(x,y)
        if self.mode == 'add_node':
            # create new node with automatic name (N1, N2...)
            i = 1
            while f"N{i}" in self.state.nodes:
                i += 1
            name = f"N{i}"
            self.push_undo()
            self.state.add_node(name, x, y)
            self.redraw()
        elif self.mode == 'add_edge':
            if clicked is None:
                return
            if self.selected_node is None:
                self.selected_node = clicked
                self.redraw()
                self.set_info(f"Selected {clicked}. Click another node to add edge.")
            else:
                a = self.selected_node; b = clicked
                if a == b:
                    self.selected_node = None
                    self.set_info("Cancelled edge creation (same node).")
                    return
                # compute euclidean distance
                dist = self.state.euclidean(a,b)
                # ask if want to override
                resp = messagebox.askyesno("Distance", f"Auto distance between {a} and {b} = {dist:.2f}. Do you want to use this distance? (No to enter custom)")
                if not resp:
                    val = simpledialog.askfloat("Distance", "Enter custom distance:", minvalue=0.0)
                    if val is None:
                        self.set_info("Edge creation cancelled.")
                        self.selected_node = None
                        return
                    dist = float(val)
                self.push_undo()
                self.state.add_edge(a,b,dist)
                self.selected_node = None
                self.set_info(f"Edge {a} <-> {b} added (dist {dist:.2f}).")
                self.redraw()
        elif self.mode == 'remove_node':
            if clicked:
                self.push_undo()
                self.state.remove_node(clicked)
                self.set_info(f"Removed node {clicked}.")
                self.redraw()
        elif self.mode == 'set_start':
            if clicked:
                self.push_undo()
                self.state.start_node = clicked
                self.set_info(f"Start node set to {clicked}")
                self.redraw()
        elif self.mode == 'set_goal':
            if clicked:
                self.push_undo()
                self.state.goal_node = clicked
                self.set_info(f"Goal node set to {clicked}")
                self.redraw()
        else:
            # default: show basic info
            if clicked:
                msg = f"Node {clicked}\nCoords: {self.state.nodes[clicked]}\nTasks at node: "
                tasks_here = [t for t,v in self.state.tasks.items() if v['location']==clicked]
                msg += ", ".join(tasks_here) if tasks_here else "None"
                messagebox.showinfo("Node Info", msg)

    def find_node_at_point(self, x,y):
        for name,(nx,ny) in self.state.nodes.items():
            if (x-nx)**2 + (y-ny)**2 <= (self.NODE_RADIUS+2)**2:
                return name
        return None

    # ---------------------------
    # Drawing helpers
    # ---------------------------
    def redraw(self):
        self.canvas.delete("all")
        # draw edges
        drawn = set()
        for a, neighs in self.state.edges.items():
            for b, d in neighs:
                if (b,a) in drawn: continue
                if a in self.state.nodes and b in self.state.nodes:
                    x1,y1 = self.state.nodes[a]; x2,y2 = self.state.nodes[b]
                    self.canvas.create_line(x1,y1,x2,y2, fill="gray", width=2)
                    mx,my = (x1+x2)/2, (y1+y2)/2
                    self.canvas.create_text(mx, my-10, text=f"{d:.2f}", fill="blue")
                    drawn.add((a,b))
        # draw nodes
        for name,(x,y) in self.state.nodes.items():
            fill="lightgreen" if name==self.state.start_node else "lightcoral" if name==self.state.goal_node else "lightblue"
            self.canvas.create_oval(x-self.NODE_RADIUS, y-self.NODE_RADIUS, x+self.NODE_RADIUS, y+self.NODE_RADIUS, fill=fill, outline="black")
            self.canvas.create_text(x, y, text=name)
            # coordinates label
            self.canvas.create_text(x + self.NODE_RADIUS + 30, y, text=f"({x},{y})", anchor="w", fill="gray", font=("Arial",8))
        # highlight selected node if in add_edge mode
        if self.mode == 'add_edge' and self.selected_node:
            x,y = self.state.nodes[self.selected_node]
            self.canvas.create_oval(x-self.NODE_RADIUS-4, y-self.NODE_RADIUS-4, x+self.NODE_RADIUS+4, y+self.NODE_RADIUS+4, outline="red", width=2)

    # ---------------------------
    # Algorithms invoked by UI
    # ---------------------------
    def run_a_star(self):
        if not self.state.start_node or not self.state.goal_node:
            messagebox.showerror("A*", "Set start and goal first.")
            return
        path, cost = a_star_shortest_distance(self.state, self.state.start_node, self.state.goal_node)
        if not path:
            messagebox.showinfo("A*", "No path found.")
            return
        # draw path
        self.redraw()
        for i in range(len(path)-1):
            a,b = path[i], path[i+1]
            x1,y1 = self.state.nodes[a]; x2,y2 = self.state.nodes[b]
            self.canvas.create_line(x1,y1,x2,y2, fill="red", width=4)
        messagebox.showinfo("A* Result", f"Path: {' → '.join(path)}\nCost: {cost:.2f}")

    def run_greedy(self):
        if not self.state.start_node or not self.state.goal_node:
            messagebox.showerror("Greedy", "Set start and goal first.")
            return
        path = greedy_path(self.state, self.state.start_node, self.state.goal_node)
        if not path:
            messagebox.showinfo("Greedy", "No path found.")
            return
        self.redraw()
        for i in range(len(path)-1):
            a,b = path[i], path[i+1]
            x1,y1 = self.state.nodes[a]; x2,y2 = self.state.nodes[b]
            self.canvas.create_line(x1,y1,x2,y2, fill="orange", width=4)
        messagebox.showinfo("Greedy Result", f"Path: {' → '.join(path)}")

    def run_csp(self):
        # Ensure tasks exist
        if not self.state.tasks:
            messagebox.showerror("CSP", "No tasks available. Add tasks in the sample data or edit code to add tasks.")
            return
        solution = csp_backtracking_solver(self.state)
        if not solution:
            messagebox.showinfo("CSP", "No valid schedule found.")
            return
        # show schedule
        s = ""
        for t, start in sorted(solution.items(), key=lambda i: i[1]):
            dur = self.state.tasks[t]['duration']
            s += f"{t} @ {self.state.tasks[t]['location']} : {start}:00 - {start+dur}:00\n"
        messagebox.showinfo("Schedule Found", s)

    def run_optimize(self):
        # Requires initial schedule; we attempt to get it from CSP, else create a naive schedule
        if not self.state.tasks:
            messagebox.showerror("Optimize", "No tasks available to optimize.")
            return
        initial = csp_backtracking_solver(self.state)
        if not initial:
            # create a simple initial schedule (non-overlapping) if CSP fails
            initial = {}
            tlist = list(self.state.tasks.keys())
            cur = WORK_START
            for t in tlist:
                initial[t] = cur
                cur += self.state.tasks[t]['duration']
                if cur > WORK_END:
                    cur = WORK_START
        best, cost = simulated_annealing_optimize(self.state, initial)
        # display
        s = "Initial schedule (from CSP or naive):\n"
        for t, st in sorted(initial.items(), key=lambda i:i[1]):
            s += f"{t} @ {self.state.tasks[t]['location']} : {st}:00 - {st + self.state.tasks[t]['duration']}:00\n"
        s += f"\nOptimized (cost {cost:.2f}):\n"
        for t, st in sorted(best.items(), key=lambda i:i[1]):
            s += f"{t} @ {self.state.tasks[t]['location']} : {st}:00 - {st + self.state.tasks[t]['duration']}:00\n"
        messagebox.showinfo("Optimization Result", s)

    # ---------------------------
    # Sample data to get started
    # ---------------------------
    def add_sample_data(self):
        self.push_undo()
        # clear first
        self.state = SmartCityState()
        # nodes
        self.state.add_node('A', 80, 120)
        self.state.add_node('B', 220, 90)
        self.state.add_node('C', 200, 240)
        self.state.add_node('D', 380, 200)
        self.state.add_node('E', 520, 120)
        self.state.add_node('F', 600, 240)
        # edges (some)
        self.state.add_edge('A','B',4)
        self.state.add_edge('A','C',2)
        self.state.add_edge('B','C',5)
        self.state.add_edge('B','D',10)
        self.state.add_edge('C','E',3)
        self.state.add_edge('D','E',4)
        self.state.add_edge('E','F',5)
        # tasks
        self.state.set_task('T1','A',1,"Pick up medical supplies")
        self.state.set_task('T2','C',2,"Deliver groceries")
        self.state.set_task('T3','E',3,"Repair power lines")
        self.state.set_task('T4','F',1,"Drop off waste")
        self.state.set_task('T5','D',2,"Inspect equipment")
        self.state.start_node = 'A'
        self.state.goal_node = 'F'
        self.redraw()
        self.set_info("Sample city loaded. Use tools to modify. Mode: add_node")

# ---------------------------
# Run application
# ---------------------------
if __name__ == "__main__":
    root = tk.Tk()
    app = SmartCityGUI(root)
    root.mainloop()
