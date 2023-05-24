using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ExamplesManager : MonoBehaviour
{
    public float DefaultSpawnExamplesDistanse = 40f;
    public List<GameObject> Examples;


    Camera cam;
    GameObject selectedExample;

    // Start is called before the first frame update
    void Start()
    {
        cam = Camera.main;
    }

    // Update is called once per frame
    void Update()
    {
        ManageExamples();
        SpawnExamples();
        if (selectedExample != null)
        {
            if (selectedExample.TryGetComponent<ExampleManager>(out var code))
            {
                code.MoveSelectedExample();
                code.RotateSelectedExample();
            }   
        }
    }

    private void SpawnExamples()
    {
        if (Input.GetKeyDown(KeyCode.Alpha1))
        {
            var point = cam.ScreenToWorldPoint(new Vector3(Input.mousePosition.x, Input.mousePosition.y, DefaultSpawnExamplesDistanse));

            if (Examples.Count > 0)
            {
                var clone = Instantiate(Examples[0]);
                clone.transform.position = point;
            }
        }
        else if (Input.GetKeyDown(KeyCode.Alpha2))
        {
            var point = cam.ScreenToWorldPoint(new Vector3(Input.mousePosition.x, Input.mousePosition.y, DefaultSpawnExamplesDistanse));

            if (Examples.Count > 1)
            {
                var clone = Instantiate(Examples[1]);
                clone.transform.position = point;
            }
        }
        else if (Input.GetKeyDown(KeyCode.Alpha3))
        {
            var point = cam.ScreenToWorldPoint(new Vector3(Input.mousePosition.x, Input.mousePosition.y, DefaultSpawnExamplesDistanse));

            if (Examples.Count > 2)
            {
                var clone = Instantiate(Examples[2]);
                clone.transform.position = point;
            }
        }
    }

    private void ManageExamples()
    {
        if (Input.GetMouseButtonDown(0))
        {
            var ray = cam.ScreenPointToRay(Input.mousePosition);

            if (Physics.Raycast(ray, out RaycastHit hit) && hit.transform.CompareTag("Examples"))
                selectedExample = hit.transform.gameObject;
        }
        else if (Input.GetMouseButtonDown(1))
            selectedExample = null;
        else if (Input.GetKeyDown(KeyCode.Delete) && selectedExample != null)
        {
            Destroy(selectedExample);
            selectedExample = null;
        }
    }
}
