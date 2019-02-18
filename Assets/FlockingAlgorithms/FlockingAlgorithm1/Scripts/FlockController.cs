using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace FlockingAlgorithm1
{

	public class FlockController : MonoBehaviour
	{

        public Transform memberboid;
        public int Numboids;
        public List<Boid> boids;
        public List<Obstacle> obs;
        public float bounds;
        public float spawnRadius;



        /// Specification Options

        public float maxFOV = 100;
        public float maxAcceleration;
        public float maxVelocity;

        //Chase Variables
        public GameObject Target;
        public float chasePriority;
       /* public float wanderJitter;
        public float WanderRadius;
        public float wanderDistance;
        public float wanderPriority;*/

        //Cohesion Variables
        public float cohesionRadius;
        public float cohesionPriority;

        //Alignment Variables
        public float alignmentRadius;
        public float alignmentPriotity;

        //Separation variables
        public float separationRadius;
        public float separationPriority;

        //Avoidance Variables
        public float avoidanceRadius;
        public float avoidancePriority;


        public Camera cam;
        Vector3 destination;
        Vector3 startPosition;
        float t;
        readonly float timeToReachTarget = 3;


        void Start()
        {
            boids = new List<Boid>();
            obs = new List<Obstacle>();

            //Iterar boids

            Spawn(memberboid, Numboids);

            boids.AddRange(FindObjectsOfType<Boid>());
            obs.AddRange(FindObjectsOfType<Obstacle>());

            
        }

        void Spawn(Transform prefab, int count)
        {
            for(int i = 0; i<count; i++)
            {
                Instantiate(prefab, new Vector3(Random.Range(-spawnRadius, spawnRadius), Random.Range(-spawnRadius, spawnRadius), 0), Quaternion.identity);

            }
        }

        public List<Boid> GetNeighbors (Boid boid, float radius)
        {
            List<Boid> neighborsFound = new List<Boid>();

            foreach(var otherboid in boids)
            {
                if (otherboid == boid)
                    continue;
                if(Vector3.Distance(boid.transform.position, otherboid.transform.position) <= radius)
                {
                    neighborsFound.Add(otherboid);
                }
                
            }
            return neighborsFound;
        }

        public List<Obstacle> GetObs(Boid boid, float radius)
        {
            List<Obstacle> obsFound = new List<Obstacle>();

            foreach (var Obstacle in obs)
            {
                if (Vector3.Distance(boid.transform.position, Obstacle.transform.position) <= radius)
                {
                    obsFound.Add(Obstacle);
                }

            }
            return obsFound;
        }

        void Update()
        {
            if (Input.GetButtonDown("Fire1"))
            {
                Target.GetComponent<Animator>().enabled = false;

                destination = cam.ScreenToWorldPoint(Input.mousePosition) - Vector3.forward * cam.transform.position.z;
                t = 0;
                startPosition = Target.transform.position;
            }

            t += Time.deltaTime / timeToReachTarget;
            Target.transform.position = Vector3.Lerp(startPosition, destination, t);
        }

    }
    


    }
