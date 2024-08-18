from devices.laser import LaserController
from flask import Flask, request, jsonify

app = Flask(__name__)
laser_controller = LaserController()

@app.route('/laser/on', methods=['POST'])
def laser_on():
    laser_controller.on()
    return jsonify({"status": "Laser turned on"}), 200

@app.route('/laser/off', methods=['POST'])
def laser_off():
    laser_controller.off()
    return jsonify({"status": "Laser turned off"}), 200

@app.route('/motor/on', methods=['POST'])
def motor_on():
    laser_controller.motor_start()
    return jsonify({"status": "Gantry motor turned on"}), 200

@app.route('/motor/off', methods=['POST'])
def motor_off():
    laser_controller.motor_stop()
    return jsonify({"status": "Motor turned off"}), 200

@app.route('/laser/move_left', methods=['POST'])
def laser_move_left():
    laser_controller.move_left()
    return jsonify({"status": "Laser moving left"}), 200

@app.route('/laser/move_right', methods=['POST'])
def laser_move_right():
    laser_controller.move_right()
    return jsonify({"status": "Laser moving right"}), 200

@app.route('/laser/cycle/enable', methods=['POST'])
def laser_cycle_enable():
    laser_controller.enable_cycle_motion()
    return jsonify({"status": "Laser cycle enabled"}), 200

@app.route('/laser/cycle/disable', methods=['POST'])
def laser_cycle_disable():
    laser_controller.disable_cycle_motion()
    return jsonify({"status": "Laser cycle disabled"}), 200
@app.route('/laser/move_to_position', methods=['POST'])
def laser_move_to_position():
    data = request.get_json()
    position = data.get('position')
    if position is None:
        return jsonify({"error": "Position not provided"}), 400
    laser_controller.move_to_position(position)
    return jsonify({"status": f"Laser moving to position {position}"}), 200

@app.route('/laser/status', methods=['GET'])
def laser_status():
    status = {
        "position": laser_controller.position,
        "max_position": laser_controller.max_position,
        "target_position": laser_controller.target_position,
        "ok_to_move_left": laser_controller.ok_to_move_left,
        "ok_to_move_right": laser_controller.ok_to_move_right
    }
    return jsonify(status), 200

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
