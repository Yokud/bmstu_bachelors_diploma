using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;

public class SceneManagerScript : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        foreach (var lightPos in EnvDataFields.LightCoords) 
        {
            Debug.Log(lightPos.ToString());
        }
    }

    // Update is called once per frame
    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Escape))
        {
            KinectManager.KinectShutdown();
            SceneManager.LoadScene("Menu");
        }
    }
}
