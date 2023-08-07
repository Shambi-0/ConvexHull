import Sift from "@rbxts/sift";

namespace ChanConvexHull {
    const EPLISON = 0.001;

    const LEFT = 1;
    const RIGHT = -1;
    const SAME = 0;

    /**
     * @description Calculate the angle between two points with respect to the x-axis.
     * @param { Vector2 } From The starting point the angle will originate from.
     * @param { Vector2 } To The point that the angle will be facing relative to the starting point.
     * @returns { number } The resulting angle in radians.
     */
    export function AngleToPoint(From: Vector2, To: Vector2): number {
        return math.atan2(From.Y - To.Y, To.X - From.X);
    };

    /**
     * @description Calculate the angle between three points, with the second being the center point.
     * @param { Vector2 } A First
     * @param { Vector2 } B Second
     * @param { Vector2 } C Third
     * @returns { number } The resulting angle in radians.
     */
    export function GetAngleFromTriangle(A: Vector2, B: Vector2, C: Vector2): number {
        const AB = math.sqrt(math.pow(B.X - A.X, 2) + math.pow(B.Y - A.Y, 2));
        const BC = math.sqrt(math.pow(B.X - C.X, 2) + math.pow(B.Y - C.Y, 2));
        const AC = math.sqrt(math.pow(C.X - A.X, 2) + math.pow(C.Y - A.Y, 2));

        return math.acos((BC * BC + AB * AB - AC * AC) / (2 * BC * AB));
    };

    /**
     * @description Based of the textbook publication, 2D area calculation between 3 points.
     * @param { Vector2 } A First
     * @param { Vector2 } B Second
     * @param { Vector2 } C Third
     * @returns { number } The resulting area between all three points.
     */
    export function GetAreaFromTriangle(A: Vector2, B: Vector2, C: Vector2): number {
        return (B.X - A.X) * (A.Y - C.Y) - (C.X - A.X) * (A.Y - B.Y);
    };

    /**
     * @description Check if the points are the same (within the elipson)
     * @param { Vector2 } [A] First
     * @param { Vector2 } [B] Second
     * @returns { boolean } If they are the same.
     */
    export function IsSamePoint(A?: Vector2, B?: Vector2): boolean {
        if (!A || !B) return false;

        return A.sub(B).Magnitude < EPLISON;
    };

    /**
     * @description Do a graham scan on the set of points provided.
     * @param { Vector2[] } Points The set of points to scan.
     * @returns { Vector2[] } The scan result.
     */
    export function GrahamScan(Points: Vector2[]): Vector2[] {
        let Lowest = Points[0];

        for (let Index = 1; Index < Points.size(); Index++) if ((Points[Index].Y > Lowest.Y) || (Points[Index].Y === Lowest.Y && Points[Index].X < Lowest.X)) {
            Lowest = Points[Index];
        };

        Points.sort((A, B) => {
            if (A.Y === Lowest.Y && A.X === Lowest.X) return false;
            if (B.Y === Lowest.Y && B.X === Lowest.X) return true;

            return AngleToPoint(Lowest, A) > AngleToPoint(Lowest, B);
        });

        {
            let Index = 0;
            while (Index < (Points.size() - 1)) {
                if (Points[Index].X === Points[Index + 1].X && Points[Index].Y === Points[Index + 1].Y) Points = Sift.Array.splice(Points, Index + 1, 0);
                Index++;
            };
        };

        const Stack: Vector2[] = [];

        if (Points.size() < 4) return Points;

        Stack[0] = Points[0];
        Stack[1] = Points[1];

        let Index = 2, Length = 2;

        while (Index < Points.size()) {
            Length = Stack.size();

            if (Length > 1) {
                if (GetAreaFromTriangle(Stack[Length - 2], Stack[Length - 1], Points[Index])) {
                    Stack.push(Points[Index]);
                    Index++;
                } else {
                    Stack.pop();
                };
            } else {
                Stack.push(Points[Index]);
                Index++;
            };
        };

        return Stack;
    };

    /**
     * @description Variation of binary search to find the tangent.
     * @param { Vector2[] } Hull The Convex Hull
     * @param { Vector2 } A First
     * @param { Vector2 } B Second
     * @returns { number } The resulting tangent.
     */
    export function TangentBinarySearch(Hull: Vector2[], A: Vector2, B: Vector2): number {
        const Length = Hull.size();

        let Start = 0, End = Length - 1, LeftSplit = -1, RightSplit = -1;
        let SearchRadius = (End - Start) + 1;

        function FindAngle(Param: number): number {
            return IsSamePoint(B, Hull[Param]) ? -999 : GetAngleFromTriangle(A, B, Hull[Param]);
        };

        if (SearchRadius === 1) {
            return 0;

        } else if (SearchRadius === 2) {
            return (FindAngle(0) > FindAngle(1)) ? 0 : 1;
        };

        while (SearchRadius > 2) {
            SearchRadius = (End - Start) + 1;

            const StartAngle = FindAngle(Start), EndAngle = FindAngle(End), Split = math.floor(SearchRadius / 2) + Start;
            let Middle = undefined;

            if (SearchRadius % 2 === 0) {
                LeftSplit = Split - 1;
                RightSplit = Split;
            } else {
                Middle = Split;
                LeftSplit = Split - 1;
                RightSplit = Split + 1;
            };

            const LeftAngle = FindAngle(LeftSplit), RightAngle = FindAngle(RightSplit), MiddleAngle = Middle ? FindAngle(Middle) : -9999;
            const MaxLeft = math.max(StartAngle, LeftAngle), MaxRight = math.max(RightAngle, EndAngle);

            if (MiddleAngle >= LeftAngle && MiddleAngle >= RightAngle) {
                return Middle!;

            } else if (MaxLeft > MaxRight) {
                End = LeftSplit;
                if (StartAngle === LeftAngle) return End;

            } else {
                Start = RightSplit;
                if (RightAngle === EndAngle) return Start;
            };
        };

        return Start;
    };

    /**
     * @description Do a Jarvis March on the prototype-hull(s) calculated by a Graham scan.
     * @param { number } March Number of iterations the algorithm should march.
     * @param { Vector2[][] } Prototypes A basic foundation for the convex hull.
     * @returns { Vector2[] } The resulting hull.
     */
    export function JarvisMarch(March: number, Prototypes: Vector2[][]): Vector2[] {
        if (Prototypes.size() === 1) return Prototypes[0];

        Prototypes.sort((A, B) => (A[0].Y < B[0].Y));

        const ConvexHull: Vector2[] = [];

        ConvexHull[0] = Prototypes[0][0];
    
        const Pointer = Vector2.yAxis.mul(ConvexHull[0].Y);

        for (let Row = 0; Row < March; Row++) {
            let MaxAngle = -math.huge, Point: Vector2 | undefined = undefined;
            const Last = (Row === 0) ? Pointer : ConvexHull[Row - 1];

            for (let Column = 0; Column < Prototypes.size(); Column++) {
                const Result = TangentBinarySearch(Prototypes[Column], Last, ConvexHull[Row]);
                const Angle = GetAngleFromTriangle(Last, ConvexHull[Row], Prototypes[Column][Result]);

                if (typeOf(Angle) === "number" && Angle > MaxAngle) {
                    MaxAngle = Angle;
                    Point = Prototypes[Column][Result];
                };
            };

            if (Point && Point.X === ConvexHull[0].X && Point.Y === ConvexHull[0].Y) return ConvexHull;

            ConvexHull.push(Point!);
        };

        return [];
    };

    /**
     * @description Partially calculates the convex hull.
     * @param { number } March The number of iterations.
     * @param { Vector2[] } Points The set of points from which the hull should be generated from.
     * @returns { Vector2[][] } An array of hull partitions.
     */
    export function CalculatePartialHulls(March: number, Points: Vector2[]): Vector2[][] {
        const Partition: Vector2[][] = [], Hulls: Vector2[][] = [];
        let Pointer = 0;

        for (let Index = 0; Index < Points.size(); Index++) {
            if (Index >= (Pointer + 1) * March) {
                Pointer++;
                Partition.push([]);
            };

            Partition[Pointer].push(Points[Index]);
        };

        for (let Index = 0; Index < Partition.size(); Index++) Hulls.push(GrahamScan(Partition[Index]));

        return Hulls;
    };

    /**
     * @description Find the convex hull of a set of points and also the partial hulls used in the final calculation.
     * @param { Vector2[] } Points The set of points that'll be used.
     * @returns { { Hull: Vector2[], Partial: Vector2[][] } } The resulting hull & it's partitions.
     */
    export function CalculateHull(Points: Vector2[]): { Hull: Vector2[], Partial: Vector2[][] } {
        let Result: Vector2[] = [], Partitions: Vector2[][] = [];

        if (Points.size() > 3) {
            let Exponent = 1;

            while (!Result) {
                const March = math.pow(2, math.pow(2, Exponent));

                Partitions = CalculatePartialHulls(March, Points);
                Result = JarvisMarch(March, Partitions);
                Exponent++;
            };
        } else {
            Result = Points;
        };

        return {
            Hull: Result,
            Partial: Partitions
        };
    };
};

export default ChanConvexHull;