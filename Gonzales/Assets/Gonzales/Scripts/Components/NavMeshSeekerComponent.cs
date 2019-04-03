using System.Collections;
using System.Collections.Generic;
using Unity.Entities;
using UnityEngine;

namespace Gonzales.Components
{
    public struct NavMeshSeeker : IComponentData
    {
    }

    public class NavMeshSeekerComponent : ComponentDataWrapper<NavMeshSeeker> { }
}

