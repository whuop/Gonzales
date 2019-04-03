using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Experimental.AI;

namespace Gonzales.Systems
{
    public class PathfindingSystem : JobComponentSystem
    {
        /// <summary>
        /// How many path queries that are allowed to run at any given time.
        /// </summary>
        private static int MAX_QUERIES = 256;
        /// <summary>
        /// Maxmium path size for each query.
        /// </summary>
        private static int MAX_PATH_SIZE = 1024;

        private static int MAX_ITERATIONS = 1024;

        private static int MAX_MAP_WIDTH = 10000;

        private static bool USE_CACHE = true;
        
        private struct Filter
        {
            public EntityArray Entities;



            public readonly int Length;
        }

        private struct UpdateQueryStatusJob : IJob
        {
            public NavMeshQuery Query;
            public PathQueryData QueryData;
            public int MaxIterations;
            public int MaxPathSize;
            public int Index;
            public NativeArray<int> Statuses;
            public NativeArray<NavMeshLocation> Results;

            public void Execute()
            {
                int performedIterations;
                var status = Query.UpdateFindPath(MaxIterations, out performedIterations);

                if (status == PathQueryStatus.InProgress | status == (PathQueryStatus.InProgress | PathQueryStatus.OutOfNodes))
                {
                    Statuses[0] = 0;
                    return;
                }

                Statuses[0] = 1;

                if (status == PathQueryStatus.Success)
                {
                    int pathSize;
                    var endStatus = Query.EndFindPath(out pathSize);
                    if (endStatus == PathQueryStatus.Success)
                    {
                        var polygons = new NativeArray<PolygonId>(pathSize, Allocator.Temp);
                        Query.GetPathResult(polygons);

                        var straightPathFlags = new NativeArray<StraightPathFlags>(MaxPathSize, Allocator.Temp);
                        var vertexSide = new NativeArray<float>(MaxPathSize, Allocator.Temp);
                        var cornerCount = 0;
                        var pathStatus = PathUtils.FindStraightPath(
                            Query,
                            QueryData.From,
                            QueryData.To,
                            polygons,
                            pathSize,
                            ref Results,
                            ref straightPathFlags,
                            ref vertexSide,
                            ref cornerCount,
                            MaxPathSize
                            );

                        if (pathStatus == PathQueryStatus.Success)
                        {
                            Statuses[1] = 1;
                            Statuses[2] = cornerCount;
                        }
                        else
                        {
                            Debug.LogWarning(pathStatus);
                            Statuses[0] = 1;
                            Statuses[1] = 2;
                        }
                        polygons.Dispose();
                        straightPathFlags.Dispose();
                        vertexSide.Dispose();
                    }
                    else
                    {
                        Debug.LogWarning(endStatus);
                        Statuses[0] = 1;
                        Statuses[1] = 2;
                    }
                }
                else
                {
                    Statuses[0] = 1;
                    Statuses[1] = 3;
                }
                
            }
        }

        [Inject]
        private Filter m_filter;
        
        private int m_cacheVersion = 0;

        private NavMeshWorld m_navWorld;
        private NavMeshQuery m_locationQuery;

        private ConcurrentQueue<PathQueryData> m_queryQueue;
        private NativeList<PathQueryData> m_progressQueue;
        private ConcurrentQueue<int> m_availableSlots;
        private List<int> m_takenSlots;
        private List<JobHandle> m_jobHandles;
        private List<NativeArray<int>> m_statuses;
        private List<NativeArray<NavMeshLocation>> m_results;
        private PathQueryData[] m_queryDatas;
        private NavMeshQuery[] m_queries;
        private Dictionary<int, UpdateQueryStatusJob> m_jobs;
        
        private ConcurrentDictionary<int, float3[]> m_cachedPaths = new ConcurrentDictionary<int, float3[]>();

        /// <summary>
        /// Delegates
        /// </summary>

        public delegate void SuccessQueryDelegate(int id, float3[] corners);
        public delegate void FailedQueryDelegate(int id, PathfindingFailedReason reason);

        private SuccessQueryDelegate m_pathResolvedCallbacks;
        private FailedQueryDelegate m_pathFailedCallbacks;

        public void RegisterPathResolvedCallback(SuccessQueryDelegate callback)
        {
            m_pathResolvedCallbacks += callback;
        }

        public void RegisterPathFailedCallback(FailedQueryDelegate callback)
        {
            m_pathFailedCallbacks += callback;
        }

        protected override void OnCreateManager()
        {
            base.OnCreateManager();
            m_navWorld = NavMeshWorld.GetDefaultWorld();
            m_locationQuery = new NavMeshQuery(m_navWorld, Allocator.Persistent);
            m_availableSlots = new ConcurrentQueue<int>();
            m_progressQueue = new NativeList<PathQueryData>(MAX_QUERIES, Allocator.Persistent);
            m_jobHandles = new List<JobHandle>(MAX_QUERIES);
            m_takenSlots = new List<int>(MAX_QUERIES);
            m_statuses = new List<NativeArray<int>>(MAX_QUERIES);
            m_results = new List<NativeArray<NavMeshLocation>>(MAX_QUERIES);
            m_jobs = new Dictionary<int, UpdateQueryStatusJob>(MAX_QUERIES);
            m_queries = new NavMeshQuery[MAX_QUERIES];
            m_queryDatas = new PathQueryData[MAX_QUERIES];
            
            for(int i = 0; i < MAX_QUERIES; i++)
            {
                m_jobHandles.Add(new JobHandle());
                m_statuses.Add(new NativeArray<int>(3, Allocator.Persistent));
                m_results.Add(new NativeArray<NavMeshLocation>(MAX_PATH_SIZE, Allocator.Persistent));
                m_availableSlots.Enqueue(i);
            }
            m_queryQueue = new ConcurrentQueue<PathQueryData>();
        }

        protected override void OnDestroyManager()
        {
            base.OnDestroyManager();
            m_progressQueue.Dispose();
            m_locationQuery.Dispose();
            for(int i = 0; i < m_takenSlots.Count; i++)
            {
                m_queries[m_takenSlots[i]].Dispose();
            }
            for(int i = 0; i < MAX_QUERIES; i++)
            {
                m_statuses[i].Dispose();
                m_results[i].Dispose();
            }
        }

        protected override JobHandle OnUpdate(JobHandle inputDeps)
        {
            //  If there are no path jobs to perform then return without doing anything.
            if (m_queryQueue.Count == 0 && m_availableSlots.Count == MAX_QUERIES)
            {
                return inputDeps;
            }

            int j = 0;
            //  Loop as long as there is work to do
            while(m_queryQueue.Count > 0 && m_availableSlots.Count > 0)
            {
                //  Try to fetch a pending path job from the Query Queue.
                PathQueryData pending;
                if (m_queryQueue.TryDequeue(out pending))
                {
                    int slotIndex;
                    float3[] waypoints;
                    //  Check if there is an already calculated path between our start and end point, by checking the path cache for our key.
                    if (USE_CACHE && m_cachedPaths.TryGetValue(pending.Key, out waypoints))
                    {
                        m_pathResolvedCallbacks?.Invoke(pending.Id, waypoints);
                    }
                    //  Calculate a new path using an available Query slot.
                    else if (m_availableSlots.TryDequeue(out slotIndex))
                    {

                    }
                    //   The slot has already been taken, lets put the pending job back in the queue for later processing.
                    else
                    {
                        Debug.Log("Path Query Slot was not available. Putting job back into queue.");
                        m_queryQueue.Enqueue(pending);
                    }
                }
                if (j > MAX_QUERIES)
                {
                    Debug.LogError("Infinite loop detected, performing a clean job shutdown. This should not happen and must be looked at!");
                    break;
                }
            }

            return base.OnUpdate(inputDeps);
        }

        /// <summary>
        /// Request a path. The ID is for you to identify the path
        /// </summary>
        /// <param name="id"></param>
        /// <param name="from"></param>
        /// <param name="to"></param>
        /// <param name="areaMask"></param>
        public void RequestPath(int id, float3 from, float3 to, int areaMask = -1)
        {
            var key = GetKey((int)from.x, (int)from.z, (int)to.x, (int)to.z);
            if (USE_CACHE)
            {
                float3[] waypoints;
                if (m_cachedPaths.TryGetValue(key, out waypoints))
                {
                    m_pathResolvedCallbacks?.Invoke(key, waypoints);
                    return;
                }
            }
            var data = new PathQueryData
            {
                Id = id,
                From = from,
                To = to,
                AreaMask = areaMask,
                Key = key
            };
            m_queryQueue.Enqueue(data);
        }

        public void PurgeCache()
        {
            m_cacheVersion++;
            m_cachedPaths.Clear();
        }
        
        private int GetKey(int fromX, int fromZ, int toX, int toZ)
        {
            var fromKey = MAX_MAP_WIDTH * fromX + fromZ;
            var toKey = MAX_MAP_WIDTH * toX + toZ;
            return MAX_MAP_WIDTH * fromKey + toKey;    
        }
    }

    public class PathUtils
    {
        public static float Perp2D(Vector3 u, Vector3 v)
        {
            return u.z * v.x - u.x * v.z;
        }

        public static void Swap(ref Vector3 a, ref Vector3 b)
        {
            var temp = a;
            a = b;
            b = temp;
        }

        // Retrace portals between corners and register if type of polygon changes
        public static int RetracePortals(NavMeshQuery query, int startIndex, int endIndex, NativeSlice<PolygonId> path, int n, Vector3 termPos, ref NativeArray<NavMeshLocation> straightPath, ref NativeArray<StraightPathFlags> straightPathFlags, int maxStraightPath)
        {
            for (var k = startIndex; k < endIndex - 1; ++k)
            {
                var type1 = query.GetPolygonType(path[k]);
                var type2 = query.GetPolygonType(path[k + 1]);
                if (type1 != type2)
                {
                    Vector3 l, r;
                    var status = query.GetPortalPoints(path[k], path[k + 1], out l, out r);
                    float3 cpa1, cpa2;
                    GeometryUtils.SegmentSegmentCPA(out cpa1, out cpa2, l, r, straightPath[n - 1].position, termPos);
                    straightPath[n] = query.CreateLocation(cpa1, path[k + 1]);

                    straightPathFlags[n] = (type2 == NavMeshPolyTypes.OffMeshConnection) ? StraightPathFlags.OffMeshConnection : 0;
                    if (++n == maxStraightPath)
                    {
                        return maxStraightPath;
                    }
                }
            }
            straightPath[n] = query.CreateLocation(termPos, path[endIndex]);
            straightPathFlags[n] = query.GetPolygonType(path[endIndex]) == NavMeshPolyTypes.OffMeshConnection ? StraightPathFlags.OffMeshConnection : 0;
            return ++n;
        }

        public static PathQueryStatus FindStraightPath(NavMeshQuery query, Vector3 startPos, Vector3 endPos, NativeSlice<PolygonId> path, int pathSize, ref NativeArray<NavMeshLocation> straightPath, ref NativeArray<StraightPathFlags> straightPathFlags, ref NativeArray<float> vertexSide, ref int straightPathCount, int maxStraightPath)
        {
            if (!query.IsValid(path[0]))
            {
                straightPath[0] = new NavMeshLocation(); // empty terminator
                return PathQueryStatus.Failure; // | PathQueryStatus.InvalidParam;
            }

            straightPath[0] = query.CreateLocation(startPos, path[0]);

            straightPathFlags[0] = StraightPathFlags.Start;

            var apexIndex = 0;
            var n = 1;

            if (pathSize > 1)
            {
                var startPolyWorldToLocal = query.PolygonWorldToLocalMatrix(path[0]);

                var apex = startPolyWorldToLocal.MultiplyPoint(startPos);
                var left = new Vector3(0, 0, 0); // Vector3.zero accesses a static readonly which does not work in burst yet
                var right = new Vector3(0, 0, 0);
                var leftIndex = -1;
                var rightIndex = -1;

                for (var i = 1; i <= pathSize; ++i)
                {
                    var polyWorldToLocal = query.PolygonWorldToLocalMatrix(path[apexIndex]);

                    Vector3 vl, vr;
                    if (i == pathSize)
                    {
                        vl = vr = polyWorldToLocal.MultiplyPoint(endPos);
                    }
                    else
                    {
                        var success = query.GetPortalPoints(path[i - 1], path[i], out vl, out vr);
                        if (!success)
                        {
                            return PathQueryStatus.Failure; // | PathQueryStatus.InvalidParam;
                        }

                        vl = polyWorldToLocal.MultiplyPoint(vl);
                        vr = polyWorldToLocal.MultiplyPoint(vr);
                    }

                    vl = vl - apex;
                    vr = vr - apex;

                    // Ensure left/right ordering
                    if (Perp2D(vl, vr) < 0)
                        Swap(ref vl, ref vr);

                    // Terminate funnel by turning
                    if (Perp2D(left, vr) < 0)
                    {
                        var polyLocalToWorld = query.PolygonLocalToWorldMatrix(path[apexIndex]);
                        var termPos = polyLocalToWorld.MultiplyPoint(apex + left);

                        n = RetracePortals(query, apexIndex, leftIndex, path, n, termPos, ref straightPath, ref straightPathFlags, maxStraightPath);
                        if (vertexSide.Length > 0)
                        {
                            vertexSide[n - 1] = -1;
                        }

                        //Debug.Log("LEFT");

                        if (n == maxStraightPath)
                        {
                            straightPathCount = n;
                            return PathQueryStatus.Success; // | PathQueryStatus.BufferTooSmall;
                        }

                        apex = polyWorldToLocal.MultiplyPoint(termPos);
                        left.Set(0, 0, 0);
                        right.Set(0, 0, 0);
                        i = apexIndex = leftIndex;
                        continue;
                    }
                    if (Perp2D(right, vl) > 0)
                    {
                        var polyLocalToWorld = query.PolygonLocalToWorldMatrix(path[apexIndex]);
                        var termPos = polyLocalToWorld.MultiplyPoint(apex + right);

                        n = RetracePortals(query, apexIndex, rightIndex, path, n, termPos, ref straightPath, ref straightPathFlags, maxStraightPath);
                        if (vertexSide.Length > 0)
                        {
                            vertexSide[n - 1] = 1;
                        }

                        //Debug.Log("RIGHT");

                        if (n == maxStraightPath)
                        {
                            straightPathCount = n;
                            return PathQueryStatus.Success; // | PathQueryStatus.BufferTooSmall;
                        }

                        apex = polyWorldToLocal.MultiplyPoint(termPos);
                        left.Set(0, 0, 0);
                        right.Set(0, 0, 0);
                        i = apexIndex = rightIndex;
                        continue;
                    }

                    // Narrow funnel
                    if (Perp2D(left, vl) >= 0)
                    {
                        left = vl;
                        leftIndex = i;
                    }
                    if (Perp2D(right, vr) <= 0)
                    {
                        right = vr;
                        rightIndex = i;
                    }
                }
            }

            // Remove the the next to last if duplicate point - e.g. start and end positions are the same
            // (in which case we have get a single point)
            if (n > 0 && (straightPath[n - 1].position == endPos))
                n--;

            n = RetracePortals(query, apexIndex, pathSize - 1, path, n, endPos, ref straightPath, ref straightPathFlags, maxStraightPath);
            if (vertexSide.Length > 0)
            {
                vertexSide[n - 1] = 0;
            }

            if (n == maxStraightPath)
            {
                straightPathCount = n;
                return PathQueryStatus.Success; // | PathQueryStatus.BufferTooSmall;
            }

            // Fix flag for final path point
            straightPathFlags[n - 1] = StraightPathFlags.End;

            straightPathCount = n;
            return PathQueryStatus.Success;
        }
    }

    [Flags]
    public enum StraightPathFlags
    {
        Start = 0x01, // The vertex is the start position.
        End = 0x02, // The vertex is the end position.
        OffMeshConnection = 0x04 // The vertex is start of an off-mesh link.
    }

    public enum PathfindingFailedReason
    {
        InvalidToOrFromLocation,
        FailedToBegin,
        FailedToResolve
    }

    public struct PathQueryData
    {
        public int Id;
        public int Key;
        public float3 From;
        public float3 To;
        public int AreaMask;
    }

    public class GeometryUtils
    {
        // Calculate the closest point of approach for line-segment vs line-segment.
        public static bool SegmentSegmentCPA(out float3 c0, out float3 c1, float3 p0, float3 p1, float3 q0, float3 q1)
        {
            var u = p1 - p0;
            var v = q1 - q0;
            var w0 = p0 - q0;

            float a = math.dot(u, u);
            float b = math.dot(u, v);
            float c = math.dot(v, v);
            float d = math.dot(u, w0);
            float e = math.dot(v, w0);

            float den = (a * c - b * b);
            float sc, tc;

            if (den == 0)
            {
                sc = 0;
                tc = d / b;

                // todo: handle b = 0 (=> a and/or c is 0)
            }
            else
            {
                sc = (b * e - c * d) / (a * c - b * b);
                tc = (a * e - b * d) / (a * c - b * b);
            }

            c0 = math.lerp(p0, p1, sc);
            c1 = math.lerp(q0, q1, tc);

            return den != 0;
        }
    }
}


