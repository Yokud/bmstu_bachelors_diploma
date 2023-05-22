using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;

public class SceneManagerScript : MonoBehaviour
{
    public GameObject LightPrefab;

    List<GameObject> prefabClones = new List<GameObject>();

    // Start is called before the first frame update
    void Start()
    {
        if (LightPrefab == null || EnvDataFields.LightCoords == null)
        {
            Debug.LogError("Light prefab is not installed or no light sources");
            return;
        }

        foreach (var lightPos in EnvDataFields.LightCoords) 
        {
            prefabClones.Add(Instantiate(LightPrefab, lightPos, Quaternion.identity));
        }
    }

    // Update is called once per frame
    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Escape))
        {
            KillLightSources();
            KinectManager.KinectShutdown();
            SceneManager.LoadScene("Menu");
        }
    }

    void KillLightSources()
    {
        foreach (var clone in prefabClones)
            Destroy(clone);
        prefabClones.Clear();
    }
}
