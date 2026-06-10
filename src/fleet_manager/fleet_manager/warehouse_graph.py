"""
Warehouse topology graph for path planning and zone management.

Represents the warehouse as a grid graph where nodes correspond to
navigable positions and edges represent valid transitions.
Used by the conflict resolver for spatiotemporal planning.
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Set, Tuple


@dataclass(frozen=True, order=True)
class Node:
    """Grid node in the warehouse graph."""
    x: float
    y: float

    def distance_to(self, other: "Node") -> float:
        return math.hypot(self.x - other.x, self.y - other.y)

    def __repr__(self) -> str:
        return f"Node({self.x:.1f}, {self.y:.1f})"


@dataclass
class Zone:
    """Named zone (dock, charging station, shelf area)."""
    name: str
    center: Node
    zone_type: str          # 'loading', 'unloading', 'charging', 'shelf', 'corridor'
    radius: float = 1.5     # Approximate zone radius in meters
    reserved_by: Optional[str] = None  # robot_id if reserved


class WarehouseGraph:
    """
    Grid-based warehouse topology.

    The warehouse is discretized into a grid with configurable resolution.
    Obstacles (shelves, walls) are marked as blocked nodes.
    Named zones (docks, chargers) provide semantic locations.
    """

    def __init__(
        self,
        width: float = 30.0,
        height: float = 20.0,
        resolution: float = 0.5,
    ) -> None:
        """
        Initialize the warehouse graph.

        Args:
            width:      Warehouse width in meters (x-axis)
            height:     Warehouse height in meters (y-axis)
            resolution: Grid cell size in meters
        """
        self.width = width
        self.height = height
        self.resolution = resolution

        # Grid bounds: x in [-width/2, width/2], y in [-height/2, height/2]
        self.x_min = -width / 2.0
        self.x_max =  width / 2.0
        self.y_min = -height / 2.0
        self.y_max =  height / 2.0

        self.blocked_nodes: Set[Node] = set()
        self.zones: Dict[str, Zone] = {}

        self._build_warehouse_layout()

    # ------------------------------------------------------------------
    # Layout Construction
    # ------------------------------------------------------------------

    def _build_warehouse_layout(self) -> None:
        """Build warehouse layout: block shelves/walls, register zones."""
        self._register_zones()
        self._block_shelves()

    def _register_zones(self) -> None:
        """Register all named zones in the warehouse."""
        zones = [
            Zone("loading_dock_1",   Node(-14.0,  3.0), "loading",   1.5),
            Zone("loading_dock_2",   Node(-14.0, -3.0), "loading",   1.5),
            Zone("unloading_dock_1", Node( 14.0,  3.0), "unloading", 1.5),
            Zone("unloading_dock_2", Node( 14.0, -3.0), "unloading", 1.5),
            Zone("charging_1",       Node(-13.0,  9.0), "charging",  1.0),
            Zone("charging_2",       Node( 13.0,  9.0), "charging",  1.0),
            Zone("charging_3",       Node(-13.0, -9.0), "charging",  1.0),
            Zone("charging_4",       Node( 13.0, -9.0), "charging",  1.0),
        ]
        # Register shelf zones
        for row_idx, row_x in enumerate([-10.0, -4.0, 4.0, 10.0]):
            for shelf_idx, shelf_y in enumerate([-8.0, -4.0, 0.0, 4.0, 8.0]):
                name = f"shelf_r{row_idx+1}_s{shelf_idx+1}"
                zones.append(Zone(name, Node(row_x, shelf_y), "shelf", 0.8))

        for zone in zones:
            self.zones[zone.name] = zone

    def _block_shelves(self) -> None:
        """Mark shelf footprints as blocked in the graph."""
        shelf_rows = [-10.0, -4.0, 4.0, 10.0]
        shelf_positions = [-8.0, -4.0, 0.0, 4.0, 8.0]

        for row_x in shelf_rows:
            for shelf_y in shelf_positions:
                # Block nodes within shelf footprint (1.0m x 0.5m + buffer)
                for dx in [-0.5, 0.0, 0.5]:
                    for dy in [-0.5, 0.0, 0.5]:
                        node = self._snap_to_grid(row_x + dx, shelf_y + dy)
                        if node:
                            self.blocked_nodes.add(node)

    # ------------------------------------------------------------------
    # Node utilities
    # ------------------------------------------------------------------

    def _snap_to_grid(self, x: float, y: float) -> Optional[Node]:
        """Snap continuous coordinate to nearest grid node, or None if OOB."""
        gx = round(x / self.resolution) * self.resolution
        gy = round(y / self.resolution) * self.resolution
        if self.x_min <= gx <= self.x_max and self.y_min <= gy <= self.y_max:
            return Node(gx, gy)
        return None

    def is_blocked(self, node: Node) -> bool:
        return node in self.blocked_nodes

    def get_neighbors(self, node: Node) -> List[Node]:
        """Return 8-connected navigable neighbors."""
        neighbors = []
        for dx in [-self.resolution, 0.0, self.resolution]:
            for dy in [-self.resolution, 0.0, self.resolution]:
                if dx == 0.0 and dy == 0.0:
                    continue
                neighbor = self._snap_to_grid(node.x + dx, node.y + dy)
                if neighbor and not self.is_blocked(neighbor):
                    neighbors.append(neighbor)
        return neighbors

    def world_to_node(self, x: float, y: float) -> Optional[Node]:
        """Convert world coordinates to nearest free grid node."""
        node = self._snap_to_grid(x, y)
        if node and not self.is_blocked(node):
            return node
        # Search nearby nodes if exact position is blocked
        for radius in [self.resolution, self.resolution * 2, self.resolution * 3]:
            for dx in [-radius, 0, radius]:
                for dy in [-radius, 0, radius]:
                    candidate = self._snap_to_grid(x + dx, y + dy)
                    if candidate and not self.is_blocked(candidate):
                        return candidate
        return None

    # ------------------------------------------------------------------
    # A* Path Planning
    # ------------------------------------------------------------------

    def find_path(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float],
        forbidden_nodes: Optional[Set[Node]] = None,
    ) -> Optional[List[Node]]:
        """
        Find shortest path using A* algorithm.

        Args:
            start:           (x, y) start position in world coordinates
            goal:            (x, y) goal position in world coordinates
            forbidden_nodes: Additional nodes to treat as blocked (for CBS)

        Returns:
            List of nodes from start to goal, or None if no path found.
        """
        start_node = self.world_to_node(*start)
        goal_node  = self.world_to_node(*goal)

        if start_node is None or goal_node is None:
            return None

        blocked = self.blocked_nodes | (forbidden_nodes or set())

        # Priority queue: (f_score, tie_breaker, node)
        open_set: List[Tuple[float, int, Node]] = []
        counter = 0
        heapq.heappush(open_set, (0.0, counter, start_node))

        came_from: Dict[Node, Optional[Node]] = {start_node: None}
        g_score: Dict[Node, float] = {start_node: 0.0}

        while open_set:
            _, _, current = heapq.heappop(open_set)

            if current == goal_node:
                return self._reconstruct_path(came_from, current)

            for neighbor in self.get_neighbors(current):
                if neighbor in blocked:
                    continue
                move_cost = current.distance_to(neighbor)
                tentative_g = g_score[current] + move_cost

                if tentative_g < g_score.get(neighbor, float("inf")):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    h = neighbor.distance_to(goal_node)
                    counter += 1
                    heapq.heappush(open_set, (tentative_g + h, counter, neighbor))

        return None  # No path found

    def _reconstruct_path(
        self,
        came_from: Dict[Node, Optional[Node]],
        current: Node,
    ) -> List[Node]:
        path = []
        while current is not None:
            path.append(current)
            current = came_from[current]
        path.reverse()
        return path

    # ------------------------------------------------------------------
    # Zone Management
    # ------------------------------------------------------------------

    def get_zone(self, zone_name: str) -> Optional[Zone]:
        return self.zones.get(zone_name)

    def get_zones_by_type(self, zone_type: str) -> List[Zone]:
        return [z for z in self.zones.values() if z.zone_type == zone_type]

    def nearest_free_charging_station(
        self,
        x: float,
        y: float,
        exclude_reserved_by: Optional[str] = None,
    ) -> Optional[Zone]:
        """Find closest charging station not reserved by another robot."""
        chargers = self.get_zones_by_type("charging")
        free = [
            c for c in chargers
            if c.reserved_by is None or c.reserved_by == exclude_reserved_by
        ]
        if not free:
            return None
        pos = Node(x, y)
        return min(free, key=lambda c: c.center.distance_to(pos))

    def reserve_zone(self, zone_name: str, robot_id: str) -> bool:
        """Reserve a zone for a robot. Returns False if already reserved."""
        zone = self.zones.get(zone_name)
        if zone is None:
            return False
        if zone.reserved_by is not None and zone.reserved_by != robot_id:
            return False
        zone.reserved_by = robot_id
        return True

    def release_zone(self, zone_name: str, robot_id: str) -> None:
        """Release a zone reservation."""
        zone = self.zones.get(zone_name)
        if zone and zone.reserved_by == robot_id:
            zone.reserved_by = None

    def path_length(self, path: List[Node]) -> float:
        """Compute total Euclidean length of a path."""
        if len(path) < 2:
            return 0.0
        return sum(path[i].distance_to(path[i + 1]) for i in range(len(path) - 1))
