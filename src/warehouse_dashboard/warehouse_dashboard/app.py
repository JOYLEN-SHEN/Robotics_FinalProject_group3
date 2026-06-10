"""
Flask + Flask-SocketIO backend for the warehouse dashboard.

Forwards ROS2 events from the queue to browser clients in real-time.
Provides REST endpoints for task submission and cancellation.
"""

from __future__ import annotations

import threading
from queue import Queue, Empty
from typing import TYPE_CHECKING

from flask import Flask, render_template_string, request, jsonify, send_from_directory
from flask_socketio import SocketIO, emit

if TYPE_CHECKING:
    from .dashboard_node import DashboardNode


def create_app(ros_node: "DashboardNode", event_queue: Queue):
    """Create and configure the Flask application."""

    import os
    # Resolve static folder relative to this file
    static_folder = os.path.join(os.path.dirname(__file__), "..", "static")
    static_folder = os.path.abspath(static_folder)

    app     = Flask(__name__, static_folder=static_folder, static_url_path="/static")
    socketio = SocketIO(app, cors_allowed_origins="*", async_mode="eventlet")

    # ------------------------------------------------------------------
    # Background thread: drain ROS2 event queue -> emit to WebSocket
    # ------------------------------------------------------------------

    def ros_event_broadcaster():
        while True:
            try:
                event = event_queue.get(timeout=0.1)
                socketio.emit(event["type"], event)
            except Empty:
                pass
            except Exception as e:
                print(f"[Dashboard] broadcaster error: {e}")

    broadcaster_thread = threading.Thread(
        target=ros_event_broadcaster, daemon=True
    )
    broadcaster_thread.start()

    # ------------------------------------------------------------------
    # Routes
    # ------------------------------------------------------------------

    @app.route("/")
    def index():
        return send_from_directory(static_folder, "index.html")

    @app.route("/api/task", methods=["POST"])
    def submit_task():
        data = request.get_json()
        try:
            task_id = ros_node.submit_task(
                pickup_x    = float(data["pickup_x"]),
                pickup_y    = float(data["pickup_y"]),
                dropoff_x   = float(data["dropoff_x"]),
                dropoff_y   = float(data["dropoff_y"]),
                pickup_zone = data.get("pickup_zone", ""),
                dropoff_zone= data.get("dropoff_zone", ""),
                priority    = int(data.get("priority", 1)),
            )
            return jsonify({"success": True, "task_id": task_id})
        except (KeyError, ValueError) as e:
            return jsonify({"success": False, "error": str(e)}), 400

    @app.route("/api/task/<task_id>", methods=["DELETE"])
    def cancel_task(task_id: str):
        success = ros_node.cancel_task(task_id)
        return jsonify({"success": success})

    @app.route("/api/health")
    def health():
        return jsonify({"status": "ok"})

    # ------------------------------------------------------------------
    # WebSocket events
    # ------------------------------------------------------------------

    @socketio.on("connect")
    def on_connect():
        print(f"[Dashboard] client connected: {request.sid}")

    @socketio.on("disconnect")
    def on_disconnect():
        print(f"[Dashboard] client disconnected: {request.sid}")

    return app, socketio
