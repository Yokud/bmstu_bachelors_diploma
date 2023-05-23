using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ExamplesManager : MonoBehaviour
{
    public float DefaultSpawnExamplesDistanse = 80f;
    public List<GameObject> Examples;
    public float MoveSpeed = 20f;
    public float RotateSpeed = 30f;

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
            MoveSelectedExample();
            RotateSelectedExample();
        }
    }

    private void SpawnExamples()
    {
        if (Input.GetKey(KeyCode.Alpha1))
        {
            var point = cam.ScreenToWorldPoint(new Vector3(Input.mousePosition.x, Input.mousePosition.y, DefaultSpawnExamplesDistanse));

            if (Examples.Count > 0)
            {
                var clone = Instantiate(Examples[0]);
                clone.transform.position = point;
            }
        }
        else if (Input.GetKey(KeyCode.Alpha2))
        {
            var point = cam.ScreenToWorldPoint(new Vector3(Input.mousePosition.x, Input.mousePosition.y, DefaultSpawnExamplesDistanse));

            if (Examples.Count > 1)
            {
                var clone = Instantiate(Examples[1]);
                clone.transform.position = point;
            }
        }
        else if (Input.GetKey(KeyCode.Alpha3))
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
        if (Input.GetMouseButton(0))
        {
            var ray = cam.ScreenPointToRay(Input.mousePosition);

            if (Physics.Raycast(ray, out RaycastHit hit) && hit.transform.CompareTag("Examples"))
                selectedExample = hit.transform.gameObject;
        }
        else if (Input.GetMouseButton(1))
            selectedExample = null;
        else if (Input.GetKey(KeyCode.Delete) && selectedExample != null)
        {
            Destroy(selectedExample);
            selectedExample = null;
        }
    }

    void MoveSelectedExample()
    {
        float deltaX = Input.GetAxis("Horizontal") * MoveSpeed;
        float deltaZ = Input.GetAxis("Vertical") * MoveSpeed;
        float deltaY = 0;

        if (Input.GetKey(KeyCode.Space))
            deltaY += MoveSpeed;
        else if (Input.GetKey(KeyCode.LeftShift))
            deltaY -= MoveSpeed;

        var movement = new Vector3(deltaX, deltaY, deltaZ);

        movement = Vector3.ClampMagnitude(movement, MoveSpeed);

        movement *= Time.deltaTime;
        selectedExample.transform.Translate(movement, Space.World);
    }

    void RotateSelectedExample()
    {
        if (Input.GetKey(KeyCode.R))
            selectedExample.transform.Rotate(RotateSpeed * Time.deltaTime * Vector3.up);
        else if (Input.GetKey(KeyCode.F))
            selectedExample.transform.Rotate(RotateSpeed * Time.deltaTime * Vector3.down);
        else if (Input.GetKey(KeyCode.Y))
            selectedExample.transform.Rotate(RotateSpeed * Time.deltaTime * Vector3.right);
        else if (Input.GetKey(KeyCode.H))
            selectedExample.transform.Rotate(RotateSpeed * Time.deltaTime * Vector3.left);
        else if (Input.GetKey(KeyCode.T))
            selectedExample.transform.Rotate(RotateSpeed * Time.deltaTime * Vector3.forward);
        else if (Input.GetKey(KeyCode.G))
            selectedExample.transform.Rotate(RotateSpeed * Time.deltaTime * Vector3.back);
    }
}
