using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace FlockingAlgorithm1
{

	//Boids is an artificial life program, developed by Craig Reynolds in 1986,
	public class Boid : MonoBehaviour
	{
        public Vector3 position;
        public Vector3 velocity;
        public Vector3 acceleration;

        public FlockController controller;

        Vector3 flockCenterPosition;
        Vector3 flockVelocity;

        Vector3 sumOfPositions = Vector3.zero;
        Vector3 sumOfVelocity = Vector3.zero;



        /*Vector3 wanderTarget;*/
        
        private void Start()
        {

            controller = FindObjectOfType<FlockController>();

            position = transform.position;
            velocity = new Vector3(Random.Range(-3, 3), Random.Range(-3, 3), 0);
        }

        void WrapAround(ref Vector3 vector, float min, float max)
        {
            vector.x = WrapAroundFloat(vector.x, min, max);
            vector.y = WrapAroundFloat(vector.y, min, max);
            vector.z = WrapAroundFloat(vector.z, min, max);

        }

        float WrapAroundFloat(float value, float min, float max)
        {
            if (value > max)
                value = min;
            else if (value < min)
                value = max;
            return value;
        }

        float RandomBinomial()
        {
            return Random.Range(0f, 1f) - Random.Range(0f, 1f);
        }


        private void Update()
        {
            acceleration = Combine();
            acceleration = Vector3.ClampMagnitude(acceleration, controller.maxAcceleration);
            velocity = velocity + acceleration * Time.deltaTime;
            velocity = Vector3.ClampMagnitude(velocity, controller.maxVelocity);
            position = position + velocity * Time.deltaTime;
            WrapAround(ref position, -controller.bounds, controller.bounds);
            transform.position = position;
        }

        /*
        protected Vector3 Wander()
        {
            float jitter = controller.wanderJitter * Time.deltaTime;
            wanderTarget += new Vector3(RandomBinomial() * jitter, RandomBinomial() * jitter, 0);
            wanderTarget = wanderTarget.normalized;
            wanderTarget *= controller.WanderRadius;
            Vector3 targetInLocalSpace = wanderTarget + new Vector3(controller.wanderDistance, controller.wanderDistance, 0);
            Vector3 targetInWorldSpace = transform.TransformPoint(targetInLocalSpace);
            targetInWorldSpace -= this.position;
            return targetInWorldSpace.normalized;
        }
        */

        Vector3 Cohesion()
        {
            Vector3 cohesionVector = new Vector3();
            int countBoids = 0;

            var neighbors = controller.GetNeighbors(this, controller.cohesionRadius);
            if(neighbors.Count == 0)
            {
                return cohesionVector;
            }
            foreach(var boid in neighbors)
            {
                if (isInFOV(boid.position))
                {
                    cohesionVector += boid.position;
                    countBoids++;
                }
            }

            if (countBoids == 0)
                return cohesionVector;

            cohesionVector /= countBoids;
            cohesionVector = cohesionVector - this.position;
            cohesionVector = Vector3.Normalize(cohesionVector);
            return cohesionVector;
        }

        Vector3 Alignment()
        {
            Vector3 alignVector = new Vector3();
            var boids = controller.GetNeighbors(this, controller.alignmentRadius);
            if (boids.Count == 0)
                return alignVector;

            foreach(var boid in boids)
            {
                if (isInFOV(boid.position))
                    alignVector += boid.velocity;
            }

            return alignVector.normalized;
        }

        Vector3 Separation()
        {
            Vector3 separateVector = new Vector3();

            var boids = controller.GetNeighbors(this, controller.separationRadius);
            if (boids.Count == 0)
                return separateVector;

            foreach (var boid in boids)
            {
                if (isInFOV(boid.position)) {
                    Vector3 movingTowards = this.position - boid.position;
                    if (movingTowards.magnitude > 0)
                    {
                        separateVector += movingTowards.normalized / movingTowards.magnitude;
                    }

                }
            }

            return separateVector.normalized;
        }

        Vector3 Avoidance()
        {
            Vector3 avoidVector = new Vector3();
            var obsList = controller.GetObs(this, controller.avoidanceRadius);
            if (obsList.Count == 0)
                return avoidVector;

            foreach(var obs in obsList)
            {
                avoidVector += RunAway(obs.position);

            }

            return avoidVector.normalized;
        }

        Vector3 Follow()
        {
            Vector3 followVector = new Vector3();
           /* List<Boid> allboids = new List<Boid>();

            allboids.AddRange(FindObjectsOfType<Boid>());
            
            foreach(var boid in allboids)
            {
                sumOfPositions += boid.transform.position;
                sumOfVelocity += boid.velocity;
            }

            flockCenterPosition = sumOfPositions / allboids.Count;
            flockVelocity = sumOfVelocity / allboids.Count;*/

            followVector = new Vector3(controller.Target.transform.position.x, controller.Target.transform.position.y, 0);

         /* flockCenterPosition = new Vector3(flockCenterPosition.x - transform.localPosition.x, flockCenterPosition.y - transform.localPosition.y);
            flockVelocity = new Vector3(flockVelocity.x - velocity.x, flockVelocity.y - velocity.y);
            */

            followVector = new Vector3(followVector.x - transform.localPosition.x, followVector.y - transform.localPosition.y);

            return followVector.normalized;
        }

        Vector3 RunAway(Vector3 target)
        {
            Vector3 neededVelocity = (position - target).normalized * controller.maxVelocity;
            return neededVelocity - velocity;
        }

        virtual protected Vector3 Combine()
        {
            Vector3 finalVec = controller.cohesionPriority * Cohesion()
                + controller.alignmentPriotity * Alignment() + controller.separationPriority * Separation() + controller.avoidancePriority * Avoidance() + Follow() * controller.chasePriority;
            return finalVec; 
        }


        bool isInFOV(Vector3 vec)
        {
            return Vector3.Angle(this.velocity, vec - this.position) <= controller.maxFOV;
        }
        /*
                private GameObject _controller;

                private bool inited = false;

                private float minVelocity;
                private float maxVelocity;

                private float randomness;
                [SerializeField]
                private GameObject chasee;
                private Rigidbody _rigidbody;

                GameObject[] boids;

                Vector3 flockCenterPosition;
                Vector3 flockVelocity;
                float neighborhoodRadius;


                float cohesionWeight;
                float alignmentWeight;
                float separationWeight;
                float desiredSeparation;
                float followWeight;


                void Start()
                {

                    StartCoroutine("BoidMovement");
                }


                Vector3 Coesion_Alignement_Separate_Rules_Calc()
                {

                    // Para cada frame
                    Vector3 sumOfPositions = Vector3.zero;
                    Vector3 sumOfVelocity = Vector3.zero;
                    Vector3 separateVelocity = Vector3.zero;

                    int countNeighbors = 0;
                    // Para cada Boid 
                    foreach (GameObject boid in boids)
                    {
                        if (Mathf.Abs(Vector3.Distance(transform.localPosition, boid.transform.position))
                            < neighborhoodRadius)
                        {
                            // faz o somatório da posição de todos os boids (1º parte da REGRA 1)
                            sumOfPositions = new Vector3(sumOfPositions.x + boid.transform.localPosition.x, sumOfPositions.y + boid.transform.localPosition.y);
                            // faz o somatório da velocidades todos os boids (1º parte da REGRA 2)
                            sumOfVelocity = new Vector3(sumOfVelocity.x + boid.GetComponent<Rigidbody>().velocity.x, sumOfVelocity.y + boid.GetComponent<Rigidbody>().velocity.y);

                            float distance = Vector3.Distance(boid.transform.localPosition, transform.localPosition);

                            if (distance > 0 && distance < desiredSeparation)
                            {

                                // o vetor da velocidade de separação será sempre oposto à velocidade de coesão, por isso o o sinal - no somatório
                                // o vetor de velocidade de separação será maior quanto mais menor for a distância (distance) entre os dois boids
                                separateVelocity = separateVelocity - (boid.transform.localPosition - transform.localPosition).normalized / distance;
                            }
                            countNeighbors++;
                        }
                    }
                    Debug.Log("nº de interations: " + countNeighbors);
                    // calcula a posição central do bando que é igual à posição média de todos os boids (2º parte da REGRA 1)
                    flockCenterPosition = new Vector3(sumOfPositions.x / (countNeighbors), sumOfPositions.y / (countNeighbors)) ;
                    // calcula a velocidade média do bando que é igual à velocidade média de todos os boids (2º parte da REGRA 2)
                    flockVelocity = new Vector3(sumOfVelocity.x / (countNeighbors), sumOfVelocity.y / (countNeighbors));

                    separateVelocity =  (separateVelocity / countNeighbors).normalized;

                    Vector3 follow = new Vector3 (chasee.transform.localPosition.x, chasee.transform.localPosition.y) ;

                    // o novo incremento da posição do bando é igual à posição central do bando menos a posição atual do boid
                    flockCenterPosition = new Vector3(flockCenterPosition.x - transform.localPosition.x , flockCenterPosition.y - transform.localPosition.y);
                    // nova velocidade do bando é igual à velocidade central do bando menos a velocidade atual do boid
                    flockVelocity = new Vector3(flockVelocity.x - _rigidbody.velocity.x, flockVelocity.y - _rigidbody.velocity.y);
                    // o novo incremento de posição para seguir a presa
                    follow = new Vector3(follow.x - transform.localPosition.x, follow.y - transform.localPosition.y);

                    // para obter um número aleatório entre -1 e 1
                    Vector3 randomize = new Vector3((Random.value * 2) - 1, (Random.value * 2) - 1, 0);

                    // Normaliza o vetor
                    randomize.Normalize();
                    // é-lhe multiplicado um fator de aleatariedade
                    randomize *= randomness;

                    Debug.DrawLine(transform.position, flockVelocity, Color.green);
                    Debug.DrawLine(transform.position, separateVelocity, Color.red);
                    Debug.DrawLine(transform.position, follow, Color.white);

                    return (cohesionWeight * flockCenterPosition + alignmentWeight * flockVelocity 
                            + separationWeight * separateVelocity + followWeight * follow 
                            + randomness * randomize);
                }


                Vector3 CollisionAvoidance()
                {
                    RaycastHit hit;
                    Vector3 Avoidance = Vector3.zero;
                    Vector3 ahead = Vector3.zero;

                        if (Physics.Raycast(transform.position, transform.forward, out hit, 20f))
                        {
                            if (hit.transform.tag == "Obstacle")
                            {
                            print("Yoooo it is hitting something");
                                ahead = transform.position + GetComponent<Rigidbody>().velocity.normalized * 20f;
                                Avoidance.x = ahead.x - hit.transform.position.x;
                                Avoidance.y = ahead.y - hit.transform.position.y;

                                Avoidance = Avoidance.normalized * 20;
                            }
                        }
                    else
                    {
                        Avoidance = Avoidance * 0;
                    }

                    return Avoidance;
                }



                // Cálculo da direção de cada boid
                IEnumerator BoidMovement()
                {

                    while (true)
                    {
                        if (inited)
                        {
                            // a sua velocidade será igual à velocidade atual + 
                            print(CollisionAvoidance());
                            _rigidbody.velocity = new Vector3(_rigidbody.velocity.x + Coesion_Alignement_Separate_Rules_Calc().x + CollisionAvoidance().x * Time.deltaTime, _rigidbody.velocity.y + Coesion_Alignement_Separate_Rules_Calc().y + CollisionAvoidance().y * Time.deltaTime);

                            // enforce minimum and maximum speeds for the boids
                            float speed = _rigidbody.velocity.magnitude;
                            // se a velocidade do boid for maior que a velocidade máxima entao a velocidade do void é igual à velocidade máxima
                            if (speed > maxVelocity)
                            {
                                _rigidbody.velocity = _rigidbody.velocity.normalized * maxVelocity;
                            }
                            // se a velocidade do boid for menor que a velocidade máxima entao a velocidade do void é igual à velocidade minima
                            else if (speed < minVelocity)
                            {
                                _rigidbody.velocity = _rigidbody.velocity.normalized * minVelocity;
                            }
                            transform.rotation = Quaternion.LookRotation(_rigidbody.velocity);

                        }
                        // espera entre .3 a .5 segundos
                        float waitTime = Random.Range(0.3f, 0.5f);
                        yield return new WaitForSeconds(waitTime);
                    }
                }


                // inicializa e liga cada boid ao BoidController em cada Boid
                public void SetController(GameObject _mycontroller)
                {

                    _controller = _mycontroller;
                    // guarda o componente numa variável por questões de performance
                    FlockController fc = _controller.GetComponent<FlockController>();
                    _rigidbody = gameObject.GetComponent<Rigidbody>();
                    boids = _controller.GetComponent<FlockController>().boids;

                    minVelocity = fc.minVelocity;
                    maxVelocity = fc.maxVelocity;		
                    randomness = fc.randomness;
                    chasee = fc.target;
                    inited = true;
                    desiredSeparation = fc.desiredSeparation;
                    cohesionWeight = fc.cohesionWeight;
                    alignmentWeight = fc.alignmentWeight;
                    separationWeight = fc.separationWeight;
                    followWeight = fc.followWeight;
                    neighborhoodRadius = fc.neighborhoodRadius;

                }

                void OnCollisionEnter(Collision collision)
                {
                    Debug.Log(" Alguém colidiu comigo!!!");
                }

            */
    }
}
