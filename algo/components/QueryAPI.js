import BaseAPI, { methodType } from "./BaseAPI";

export default class QueryAPI extends BaseAPI {
  /**
   * Runs the simulation by sending a request to the backend.
   * This is an async function that returns a promise.
   * @param {object} config The simulation configuration.
   * @returns {Promise<object>} The response from the backend.
   */
  static async runSimulation(config) {
    const content = {
      ...config,
      retrying: false, // Assuming this is always false for a new simulation
    };

    // Send the request to the backend server and return the promise
    return this.JSONRequest("/path", methodType.post, {}, {}, content);
  }

  // Query the path from backend server (legacy callback-based method)
  static query(obstacles, robotX, robotY, robotDir, callback) {
    /* Construct the content of the request 
		obstacles: the array of obstacles
		robotX: the x coordinate of the robot
		robotY: the y coordinate of the robot
		robotDir: the direction of the robot
		retrying: whether the robot is retrying
	*/
    const content = {
      obstacles: obstacles,
      robot_x: robotX,
      robot_y: robotY,
      robot_dir: robotDir,
      retrying: false,
    };

    // Send the request to the backend server
    this.JSONRequest("/path", methodType.post, {}, {}, content)
      .then((res) => {
        if (callback) {
          // The original implementation had a bug here, wrapping res in another data object.
          // The new runSimulation method returns the direct response.
          callback({
            data: res,
            error: null,
          });
        }
      })
      .catch((err) => {
        console.log(err);
        if (callback) {
          callback({
            data: null,
            error: err,
          });
        }
      });
  }
}
