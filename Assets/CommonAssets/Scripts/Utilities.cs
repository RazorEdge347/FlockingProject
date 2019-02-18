using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Utilities : MonoBehaviour {


	public static List<GameObject> GetObjectsInLayer(GameObject root, int layer)
	{
		List<GameObject> obstacleList = new List<GameObject>();

		for (int childIndex = 0; childIndex < root.transform.childCount; ++childIndex) {

			GameObject _object = root.transform.GetChild (childIndex).gameObject;

			if (_object.layer == layer) {
				obstacleList.Add (root.transform.GetChild (childIndex).gameObject);
			}
		}	
		return obstacleList;        
	}


	public static List<GameObject> GetChildren(GameObject root) {

		List<GameObject> childList = new List<GameObject>();

		for (int childIndex = 0; childIndex < root.transform.childCount; ++childIndex) {

			GameObject _object = root.transform.GetChild (childIndex).gameObject;

			childList.Add (_object);

		}	
		return childList;    	
	}

	public static List<GameObject> GetChildren(GameObject root, string tag) {

		List<GameObject> childList = new List<GameObject>();

		foreach(GameObject g in childList) {
			if (g.transform.tag == tag) {
				childList.Add (g);
			}
		}	
		return childList;    	
	}
}