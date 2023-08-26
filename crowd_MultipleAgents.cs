using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

public class crowd_MultipleAgents : Agent
{
    
    [SerializeField] private Transform targetTransform; //goal position
    [SerializeField] private Transform centerTransform; //centre of local environment
    [SerializeField] private Material winMaterial; //Green material
    [SerializeField] private Material loseMaterial; //Red material
    [SerializeField] private MeshRenderer floorMesh; //Floor

    //Obstacle lists for spawning and position
    [SerializeField] private List<GameObject> spawnedObstacles = new List<GameObject>();
    [SerializeField] private GameObject Obstacle;
    [SerializeField] private List<Transform> spawnedObstacleTransforms = new List<Transform>();

    //Init velovity
    [SerializeField] private Vector3 currentVelocity = Vector3.zero;

    //Count for number of obstacles
    public int numberOfDuplicates = 0;

    //Generates obstacles in game world, setting position to a local randomly generated set of coords. 
    //Use of .localPosition required so they spawn in their environment only.
    void RegenerateObstacles()
    {

        //Generate new obstacles
        for (int i = 0; i < numberOfDuplicates; i++)
        {

            GameObject duplicateObstacle = Instantiate(Obstacle);
            duplicateObstacle.transform.parent = centerTransform;
            duplicateObstacle.transform.localPosition = GenerateRandomPosition();
            spawnedObstacles.Add(duplicateObstacle);
            spawnedObstacleTransforms.Add(duplicateObstacle.transform);

        }

    }

    //Delete spawned obstacles from environment and clear lists
    void ClearSpawnedObjects()
    {
        foreach (GameObject obstacle in spawnedObstacles)
        {
            Destroy(obstacle);
        }

        spawnedObstacles.Clear();
        spawnedObstacleTransforms.Clear();
    }

    //Each episode cleans the envionment and sets new random positions for agent + goal, then generates obstacles.
    public override void OnEpisodeBegin()
    {
        ClearSpawnedObjects();

        //Comment below to disable random spawning


        transform.localPosition = GenerateRandomPosition();
        targetTransform.localPosition = GenerateRandomPosition();
        
        RegenerateObstacles();


    }

    //Random position in a given range, this range is altered to work for the size of the given environment.
    Vector3 GenerateRandomPosition()
    {
        return new Vector3(Random.Range(-15f, 15f), 0, Random.Range(-15f, 15f));
    }

    //Gather states/obnservations
    public override void CollectObservations(VectorSensor sensor)
    {
        //Get agent and goal position
        sensor.AddObservation(transform.localPosition);
        sensor.AddObservation(targetTransform.localPosition);
        //Calculate the distance to goal
        float distanceToGoal = Vector3.Distance(transform.localPosition, targetTransform.localPosition);
        sensor.AddObservation(distanceToGoal);

        //Add objects position observations
        foreach (Transform obstacle in spawnedObstacleTransforms)
        {
            sensor.AddObservation(obstacle.localPosition);
        }

        //Other observations added via components on agent gameObject via Unity.
    }

    //Actions: Movement x/y altered to give more realistic speed and turning
    public override void OnActionReceived(ActionBuffers actions)
    {

        float moveSpeed = 3f;
        float obstacleAvoidanceDistance = 0.5f;

        float moveX = Mathf.Clamp(actions.ContinuousActions[0], -3f, 3f);
        float moveZ = Mathf.Clamp(actions.ContinuousActions[1], -3f, 3f);

        Vector3 moveDirection = new Vector3(moveX, 0, moveZ).normalized;
        Vector3 newVelocity = moveDirection * moveSpeed;

        if (Physics.Raycast(transform.position, newVelocity, out RaycastHit hit, obstacleAvoidanceDistance))
        { 
            moveDirection = Vector3.Reflect(moveDirection, hit.normal);
            newVelocity = moveDirection * moveSpeed;
        }

        // Smooth the change in velocity
        currentVelocity = Vector3.Lerp(currentVelocity, newVelocity, 0.5f * Time.deltaTime);

        // Update the position using the smoothed velocity
        transform.localPosition += currentVelocity * Time.deltaTime;
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        //Setting manual controls for actions to arrow keys/W A S D.
        ActionSegment<float> continuousActions = actionsOut.ContinuousActions;
        continuousActions[0] = Input.GetAxisRaw("Horizontal");
        continuousActions[1] = Input.GetAxisRaw("Vertical");
    }

    //Unity monitors collisions via below
    private void OnTriggerEnter(Collider other)
    {
        //Setting rewards/materials and episode end/obstacle regeneration for different collisions.
        if (other.TryGetComponent(out Goal Goal))
        {
            SetReward(5f);
            floorMesh.material = winMaterial;
            EndEpisode();
            RegenerateObstacles();
        }

        if (other.TryGetComponent(out Wall Wall))
        {
            SetReward(-1f);
            floorMesh.material = loseMaterial;
            EndEpisode();
            RegenerateObstacles();
        }

        if (other.TryGetComponent(out agent agent))
        {
            SetReward(-3f);
        }

        if (other.TryGetComponent(out Obstacle Obstacle))
        {
            SetReward(-.5f);
            floorMesh.material = loseMaterial;
            EndEpisode();
            RegenerateObstacles();
        }




    }

}
