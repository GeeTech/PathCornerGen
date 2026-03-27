using System.Drawing.Drawing2D;

namespace BezierFilletTool;

public enum SegmentKind
{
    Line,
    Bezier
}

public enum JoinStyle
{
    RoundFillet,
    FlatCut
}

public class BezierSegment
{
    public PointF P0;
    public PointF P1;
    public PointF P2;
    public PointF P3;

    public BezierSegment() { }

    public BezierSegment(PointF p0, PointF p1, PointF p2, PointF p3)
    {
        P0 = p0;
        P1 = p1;
        P2 = p2;
        P3 = p3;
    }
}

public class PathSegment
{
    public SegmentKind Kind;
    public PointF P0;
    public PointF P1;
    public PointF P2;
    public PointF P3;

    public static PathSegment FromBezier(BezierSegment b) => new()
    {
        Kind = SegmentKind.Bezier,
        P0 = b.P0,
        P1 = b.P1,
        P2 = b.P2,
        P3 = b.P3
    };

    public static PathSegment FromLine(PointF p0, PointF p1) => new()
    {
        Kind = SegmentKind.Line,
        P0 = p0,
        P1 = p0,
        P2 = p1,
        P3 = p1
    };

    public BezierSegment AsBezier()
    {
        if (Kind == SegmentKind.Bezier) return new BezierSegment(P0, P1, P2, P3);

        // 直线转三次 Bezier 的精确表示
        PointF c1 = BezierMath.Lerp(P0, P3, 1f / 3f);
        PointF c2 = BezierMath.Lerp(P0, P3, 2f / 3f);
        return new BezierSegment(P0, c1, c2, P3);
    }
}

public static class BezierMath
{
    private sealed class NodeSolveResult
    {
        public float TPrev;
        public float TNext;
        public PointF Center;
        public float Error;
    }

    // 兼容原始 API：保持纯 Bezier 调用方式
    public static PointF Evaluate(BezierSegment s, float t) => Evaluate(PathSegment.FromBezier(s), t);

    public static PointF Derivative(BezierSegment s, float t) => Derivative(PathSegment.FromBezier(s), t);

    public static (BezierSegment left, BezierSegment right) Split(BezierSegment s, float t)
    {
        var (left, right) = Split(PathSegment.FromBezier(s), t);
        return (left.AsBezier(), right.AsBezier());
    }

    public static PointF Evaluate(PathSegment s, float t)
    {
        if (s.Kind == SegmentKind.Line)
        {
            // 直线参数方程：P(t)=P0+t*(P3-P0)
            return Lerp(s.P0, s.P3, t);
        }

        float u = 1f - t;
        float b0 = u * u * u;
        float b1 = 3f * u * u * t;
        float b2 = 3f * u * t * t;
        float b3 = t * t * t;
        return Add(Scale(s.P0, b0), Add(Scale(s.P1, b1), Add(Scale(s.P2, b2), Scale(s.P3, b3))));
    }

    public static PointF Derivative(PathSegment s, float t)
    {
        if (s.Kind == SegmentKind.Line)
        {
            // 直线导数恒定
            return Sub(s.P3, s.P0);
        }

        float u = 1f - t;
        PointF a = Scale(Sub(s.P1, s.P0), 3f * u * u);
        PointF b = Scale(Sub(s.P2, s.P1), 6f * u * t);
        PointF c = Scale(Sub(s.P3, s.P2), 3f * t * t);
        return Add(a, Add(b, c));
    }

    public static (PathSegment left, PathSegment right) Split(PathSegment s, float t)
    {
        if (s.Kind == SegmentKind.Line)
        {
            // 直线分割：取参数点 m 并拆成两段
            PointF m = Lerp(s.P0, s.P3, t);
            return (PathSegment.FromLine(s.P0, m), PathSegment.FromLine(m, s.P3));
        }

        // 三次 Bezier：de Casteljau 分割
        PointF p01 = Lerp(s.P0, s.P1, t);
        PointF p12 = Lerp(s.P1, s.P2, t);
        PointF p23 = Lerp(s.P2, s.P3, t);
        PointF p012 = Lerp(p01, p12, t);
        PointF p123 = Lerp(p12, p23, t);
        PointF p0123 = Lerp(p012, p123, t);

        return (
            new PathSegment { Kind = SegmentKind.Bezier, P0 = s.P0, P1 = p01, P2 = p012, P3 = p0123 },
            new PathSegment { Kind = SegmentKind.Bezier, P0 = p0123, P1 = p123, P2 = p23, P3 = s.P3 }
        );
    }

    public static List<BezierSegment> ApplyFillet(BezierSegment seg1, BezierSegment seg2, float radius, JoinStyle style = JoinStyle.RoundFillet)
    {
        var raw = ApplyFillet(PathSegment.FromBezier(seg1), PathSegment.FromBezier(seg2), radius, style);
        return raw.Select(s => s.AsBezier()).ToList();
    }

    public static List<PathSegment> ApplyFillet(PathSegment seg1, PathSegment seg2, float radius, JoinStyle style = JoinStyle.RoundFillet)
    {
        // 关键方程：在两段上寻找 t1/t2，使“法线偏移半径 r”后的两圆心重合
        const float eps = 1e-4f;
        var solutions = new List<(float t1, float t2, int side, PointF c, float err)>();

        // 双侧法线都尝试，解决拐角内外侧不确定性
        foreach (int side in new[] { 1, -1 })
        {
            if (TrySolve(seg1, seg2, radius, side, out float t1, out float t2, out PointF center, out float err))
            {
                if (t1 > eps && t1 < 1f - eps && t2 > eps && t2 < 1f - eps)
                {
                    solutions.Add((t1, t2, side, center, err));
                }
            }
        }

        if (solutions.Count == 0)
        {
            // 无可行解则回退到原路径
            return new List<PathSegment> { seg1, seg2 };
        }

        var best = solutions.OrderBy(s => s.err + (1f - s.t1) + s.t2).First();
        var (s1Left, _) = Split(seg1, best.t1);
        var (_, s2Right) = Split(seg2, best.t2);

        PointF pStart = Evaluate(seg1, best.t1);
        PointF pEnd = Evaluate(seg2, best.t2);
        PointF dStart = Normalize(Derivative(seg1, best.t1));
        PointF dEnd = Normalize(Derivative(seg2, best.t2));

        // 连接段可切换为：圆角弧线 / 平头直线
        List<PathSegment> joinSegments = style == JoinStyle.FlatCut
            ? BuildFlatPath(pStart, pEnd)
            : BuildArcPath(pStart, pEnd, best.c, radius, dStart, dEnd);

        var result = new List<PathSegment> { s1Left };
        result.AddRange(joinSegments);
        result.Add(s2Right);
        return result;
    }

    /// <summary>
    /// 多段路径统一圆角（或平头）处理：
    /// 1) 优先尝试用户给定半径；
    /// 2) 若存在节点无解/相邻截断冲突，则二分半径；
    /// 3) 返回全局可行的最大半径 effectiveRadius。
    /// </summary>
    public static List<PathSegment> ApplyFilletForPath(
        IReadOnlyList<PathSegment> segments,
        float radius,
        JoinStyle style,
        out float effectiveRadius,
        bool closed = false)
    {
        effectiveRadius = 0f;
        if (segments.Count == 0) return new List<PathSegment>();
        if (segments.Count == 1 || radius <= 0f)
        {
            return segments.Select(Clone).ToList();
        }

        if (TryBuildPathWithRadius(segments, radius, style, closed, out var exact))
        {
            effectiveRadius = radius;
            return exact;
        }

        float low = 0f, high = radius;
        List<PathSegment> best = segments.Select(Clone).ToList();
        for (int i = 0; i < 24; i++)
        {
            float mid = (low + high) * 0.5f;
            if (mid < 0.01f) break;

            if (TryBuildPathWithRadius(segments, mid, style, closed, out var candidate))
            {
                low = mid;
                best = candidate;
            }
            else
            {
                high = mid;
            }
        }

        effectiveRadius = low;
        return best;
    }

    private static bool TryBuildPathWithRadius(
        IReadOnlyList<PathSegment> segments,
        float radius,
        JoinStyle style,
        bool closed,
        out List<PathSegment> output)
    {
        output = new List<PathSegment>();
        int segCount = segments.Count;
        int nodeCount = closed ? segCount : segCount - 1;
        if (nodeCount <= 0)
        {
            output = segments.Select(Clone).ToList();
            return true;
        }

        var nodes = new NodeSolveResult[nodeCount];
        for (int i = 0; i < nodeCount; i++)
        {
            PathSegment prev = segments[i];
            PathSegment next = segments[(i + 1) % segCount];
            if (!TrySolveNode(prev, next, radius, out NodeSolveResult solved))
            {
                return false;
            }
            nodes[i] = solved;
        }

        float eps = 1e-4f;
        for (int s = 0; s < segCount; s++)
        {
            float start = 0f;
            float end = 1f;
            if (closed || s > 0)
            {
                int prevNode = (s - 1 + nodeCount) % nodeCount;
                start = nodes[prevNode].TNext;
            }
            if (closed || s < segCount - 1)
            {
                int node = s % nodeCount;
                end = nodes[node].TPrev;
            }
            if (end - start <= eps)
            {
                return false;
            }
        }

        for (int s = 0; s < segCount; s++)
        {
            float start = 0f;
            float end = 1f;
            if (closed || s > 0)
            {
                int prevNode = (s - 1 + nodeCount) % nodeCount;
                start = nodes[prevNode].TNext;
            }
            if (closed || s < segCount - 1)
            {
                int node = s % nodeCount;
                end = nodes[node].TPrev;
            }

            output.Add(ExtractSubSegment(segments[s], start, end));

            if (s < nodeCount)
            {
                var n = nodes[s];
                PointF pStart = Evaluate(segments[s], n.TPrev);
                PointF pEnd = Evaluate(segments[(s + 1) % segCount], n.TNext);
                PointF dStart = Normalize(Derivative(segments[s], n.TPrev));
                PointF dEnd = Normalize(Derivative(segments[(s + 1) % segCount], n.TNext));

                List<PathSegment> join = style == JoinStyle.FlatCut
                    ? BuildFlatPath(pStart, pEnd)
                    : BuildArcPath(pStart, pEnd, n.Center, radius, dStart, dEnd);
                output.AddRange(join);
            }
        }

        if (closed && output.Count > 0)
        {
            // 闭环场景中上面循环已为每个节点追加连接段；
            // 若需要严格闭合，可在渲染时使用 GraphicsPath.CloseFigure。
        }
        return true;
    }

    private static PathSegment Clone(PathSegment s) =>
        new() { Kind = s.Kind, P0 = s.P0, P1 = s.P1, P2 = s.P2, P3 = s.P3 };

    private static PathSegment ExtractSubSegment(PathSegment s, float tStart, float tEnd)
    {
        tStart = Clamp(tStart, 0f, 1f);
        tEnd = Clamp(tEnd, 0f, 1f);
        if (tEnd <= tStart) return Clone(s);
        if (tStart <= 1e-6f && tEnd >= 1f - 1e-6f) return Clone(s);

        var (_, right) = Split(s, tStart);
        float localT = (tEnd - tStart) / (1f - tStart);
        var (middle, _) = Split(right, localT);
        return middle;
    }

    private static bool TrySolveNode(PathSegment seg1, PathSegment seg2, float radius, out NodeSolveResult best)
    {
        const float eps = 1e-4f;
        var solutions = new List<NodeSolveResult>();
        foreach (int side in new[] { 1, -1 })
        {
            if (TrySolve(seg1, seg2, radius, side, out float t1, out float t2, out PointF center, out float err))
            {
                if (t1 > eps && t1 < 1f - eps && t2 > eps && t2 < 1f - eps)
                {
                    solutions.Add(new NodeSolveResult { TPrev = t1, TNext = t2, Center = center, Error = err });
                }
            }
        }

        if (solutions.Count == 0)
        {
            best = new NodeSolveResult();
            return false;
        }

        best = solutions.OrderBy(s => s.Error + (1f - s.TPrev) + s.TNext).First();
        return true;
    }

    private static List<PathSegment> BuildFlatPath(PointF pStart, PointF pEnd)
    {
        // 平头样式：直接用两个切点连线替代圆弧
        return new List<PathSegment> { PathSegment.FromLine(pStart, pEnd) };
    }

    private static bool TrySolve(PathSegment seg1, PathSegment seg2, float r, int side,
        out float t1, out float t2, out PointF center, out float err)
    {
        // 初值靠近节点（seg1 末端附近、seg2 起点附近）
        t1 = seg1.Kind == SegmentKind.Line ? 0.8f : 0.9f;
        t2 = seg2.Kind == SegmentKind.Line ? 0.2f : 0.1f;
        center = new PointF();
        err = float.MaxValue;

        for (int i = 0; i < 40; i++)
        {
            PointF p1 = Evaluate(seg1, t1);
            PointF p2 = Evaluate(seg2, t2);
            PointF d1 = Normalize(Derivative(seg1, t1));
            PointF d2 = Normalize(Derivative(seg2, t2));
            if (Length(d1) < 1e-6f || Length(d2) < 1e-6f) return false;

            PointF n1 = Scale(LeftNormal(d1), side);
            PointF n2 = Scale(LeftNormal(d2), side);
            PointF c1 = Add(p1, Scale(n1, r));
            PointF c2 = Add(p2, Scale(n2, r));
            PointF f = Sub(c1, c2);
            // 残差 f=c1-c2，目标是逼近零向量
            err = Length(f);
            center = Mid(c1, c2);
            if (err < 0.5f) return true;

            // 数值雅可比 + 牛顿迭代步
            float h = 1e-3f;
            var f_t1 = Residual(seg1, seg2, r, side, t1 + h, t2);
            var f_t2 = Residual(seg1, seg2, r, side, t1, t2 + h);
            float j11 = (f_t1.X - f.X) / h;
            float j21 = (f_t1.Y - f.Y) / h;
            float j12 = (f_t2.X - f.X) / h;
            float j22 = (f_t2.Y - f.Y) / h;

            float det = j11 * j22 - j12 * j21;
            if (MathF.Abs(det) < 1e-8f) return false;

            float dt1 = (-f.X * j22 + f.Y * j12) / det;
            float dt2 = (-j11 * f.Y + j21 * f.X) / det;

            // 阻尼线搜索，提升收敛稳定性
            float damping = 1f;
            bool accepted = false;
            for (int k = 0; k < 6; k++)
            {
                float nt1 = Clamp(t1 + damping * dt1, 0.001f, 0.999f);
                float nt2 = Clamp(t2 + damping * dt2, 0.001f, 0.999f);
                PointF nf = Residual(seg1, seg2, r, side, nt1, nt2);
                if (Length(nf) < err)
                {
                    t1 = nt1;
                    t2 = nt2;
                    accepted = true;
                    break;
                }
                damping *= 0.5f;
            }

            if (!accepted) return false;
        }

        return err < 1f;
    }

    private static PointF Residual(PathSegment seg1, PathSegment seg2, float r, int side, float t1, float t2)
    {
        // F(t1,t2)=c1-c2
        PointF p1 = Evaluate(seg1, t1);
        PointF p2 = Evaluate(seg2, t2);
        PointF d1 = Normalize(Derivative(seg1, t1));
        PointF d2 = Normalize(Derivative(seg2, t2));
        PointF c1 = Add(p1, Scale(Scale(LeftNormal(d1), side), r));
        PointF c2 = Add(p2, Scale(Scale(LeftNormal(d2), side), r));
        return Sub(c1, c2);
    }

    private static List<PathSegment> BuildArcPath(PointF pStart, PointF pEnd, PointF center, float radius, PointF dStart, PointF dEnd)
    {
        // 圆心 + 起止点 -> 极角，再根据切向匹配确定顺/逆时针
        float a1 = MathF.Atan2(pStart.Y - center.Y, pStart.X - center.X);
        float a2 = MathF.Atan2(pEnd.Y - center.Y, pEnd.X - center.X);

        float deltaCcw = NormalizeAnglePositive(a2 - a1);
        float deltaCw = deltaCcw - 2f * MathF.PI;

        float scoreCcw = Dot(TangentFromAngle(a1, true), dStart) + Dot(TangentFromAngle(a2, true), dEnd);
        float scoreCw = Dot(TangentFromAngle(a1, false), dStart) + Dot(TangentFromAngle(a2, false), dEnd);

        float delta = scoreCcw >= scoreCw ? deltaCcw : deltaCw;
        // 拆分为每段 <= 90°，提高圆弧三次 Bezier 近似精度
        int pieces = Math.Max(1, (int)MathF.Ceiling(MathF.Abs(delta) / (MathF.PI / 2f)));
        float step = delta / pieces;

        var arc = new List<PathSegment>();
        float current = a1;
        for (int i = 0; i < pieces; i++)
        {
            float next = current + step;
            float theta = next - current;
            // 圆弧 Bezier 近似系数
            float k = 4f / 3f * MathF.Tan(MathF.Abs(theta) / 4f);
            int sign = theta >= 0 ? 1 : -1;

            PointF r0 = new(MathF.Cos(current), MathF.Sin(current));
            PointF r1 = new(MathF.Cos(next), MathF.Sin(next));
            PointF p0 = Add(center, Scale(r0, radius));
            PointF p3 = Add(center, Scale(r1, radius));

            PointF t0 = sign > 0 ? new PointF(-r0.Y, r0.X) : new PointF(r0.Y, -r0.X);
            PointF t1 = sign > 0 ? new PointF(-r1.Y, r1.X) : new PointF(r1.Y, -r1.X);
            PointF c1 = Add(p0, Scale(t0, k * radius));
            PointF c2 = Sub(p3, Scale(t1, k * radius));

            arc.Add(new PathSegment { Kind = SegmentKind.Bezier, P0 = p0, P1 = c1, P2 = c2, P3 = p3 });
            current = next;
        }

        arc[0].P0 = pStart;
        arc[^1].P3 = pEnd;
        return arc;
    }

    private static PointF TangentFromAngle(float angle, bool ccw)
    {
        float y = MathF.Sin(angle);
        float c = MathF.Cos(angle);
        return ccw ? new PointF(-y, c) : new PointF(y, -c);
    }

    private static float NormalizeAnglePositive(float a)
    {
        while (a < 0) a += 2f * MathF.PI;
        while (a >= 2f * MathF.PI) a -= 2f * MathF.PI;
        return a;
    }

    public static PointF Add(PointF a, PointF b) => new(a.X + b.X, a.Y + b.Y);
    public static PointF Sub(PointF a, PointF b) => new(a.X - b.X, a.Y - b.Y);
    public static PointF Scale(PointF a, float s) => new(a.X * s, a.Y * s);
    public static float Dot(PointF a, PointF b) => a.X * b.X + a.Y * b.Y;
    public static float Length(PointF a) => MathF.Sqrt(a.X * a.X + a.Y * a.Y);
    public static PointF Normalize(PointF a)
    {
        float l = Length(a);
        if (l < 1e-8f) return new PointF(0, 0);
        return Scale(a, 1f / l);
    }
    public static PointF LeftNormal(PointF d) => new(-d.Y, d.X);
    public static float Clamp(float x, float min, float max) => MathF.Max(min, MathF.Min(max, x));
    public static PointF Lerp(PointF a, PointF b, float t) => Add(a, Scale(Sub(b, a), t));
    public static PointF Mid(PointF a, PointF b) => new((a.X + b.X) * 0.5f, (a.Y + b.Y) * 0.5f);
}

public sealed class MainForm : Form
{
    private readonly Panel _canvas = new() { Dock = DockStyle.Fill, BackColor = Color.White };
    private readonly TrackBar _radiusTrack = new() { Dock = DockStyle.Top, Minimum = 5, Maximum = 180, Value = 40, TickFrequency = 5, Height = 45 };
    private readonly Label _info = new() { Dock = DockStyle.Top, Height = 24, TextAlign = ContentAlignment.MiddleLeft };
    private readonly CheckBox _seg1Line = new() { Dock = DockStyle.Top, Height = 24, Text = "第一段为直线" };
    private readonly CheckBox _seg2Line = new() { Dock = DockStyle.Top, Height = 24, Text = "第二段为直线" };
    private readonly CheckBox _flatCut = new() { Dock = DockStyle.Top, Height = 24, Text = "使用平头直线样式(替换圆角)" };

    private readonly List<PointF> _ctrl = new();
    private int _dragIndex = -1;

    public MainForm()
    {
        Text = "Bezier/Line 圆角 (Fillet) 工具";
        Width = 1000;
        Height = 720;

        _ctrl.AddRange(new[]
        {
            new PointF(80, 420),
            new PointF(250, 150),
            new PointF(420, 240),
            new PointF(500, 320),
            new PointF(590, 420),
            new PointF(750, 180),
            new PointF(910, 360),
        });

        Controls.Add(_canvas);
        Controls.Add(_info);
        Controls.Add(_flatCut);
        Controls.Add(_seg2Line);
        Controls.Add(_seg1Line);
        Controls.Add(_radiusTrack);

        _radiusTrack.ValueChanged += (_, _) => _canvas.Invalidate();
        _seg1Line.CheckedChanged += (_, _) => _canvas.Invalidate();
        _seg2Line.CheckedChanged += (_, _) => _canvas.Invalidate();
        _flatCut.CheckedChanged += (_, _) => _canvas.Invalidate();
        _canvas.Paint += CanvasOnPaint;
        _canvas.MouseDown += CanvasOnMouseDown;
        _canvas.MouseMove += CanvasOnMouseMove;
        _canvas.MouseUp += (_, _) => _dragIndex = -1;
    }

    private void CanvasOnMouseDown(object? sender, MouseEventArgs e)
    {
        for (int i = 0; i < _ctrl.Count; i++)
        {
            if (BezierMath.Length(BezierMath.Sub(_ctrl[i], e.Location)) < 10f)
            {
                if ((_seg1Line.Checked && (i == 1 || i == 2)) || (_seg2Line.Checked && (i == 4 || i == 5)))
                {
                    continue;
                }
                _dragIndex = i;
                return;
            }
        }
    }

    private void CanvasOnMouseMove(object? sender, MouseEventArgs e)
    {
        if (_dragIndex < 0) return;
        _ctrl[_dragIndex] = e.Location;
        _canvas.Invalidate();
    }

    private void CanvasOnPaint(object? sender, PaintEventArgs e)
    {
        e.Graphics.SmoothingMode = SmoothingMode.AntiAlias;

        var seg1 = _seg1Line.Checked
            ? PathSegment.FromLine(_ctrl[0], _ctrl[3])
            : new PathSegment { Kind = SegmentKind.Bezier, P0 = _ctrl[0], P1 = _ctrl[1], P2 = _ctrl[2], P3 = _ctrl[3] };

        var seg2 = _seg2Line.Checked
            ? PathSegment.FromLine(_ctrl[3], _ctrl[6])
            : new PathSegment { Kind = SegmentKind.Bezier, P0 = _ctrl[3], P1 = _ctrl[4], P2 = _ctrl[5], P3 = _ctrl[6] };

        float r = _radiusTrack.Value;
        JoinStyle style = _flatCut.Checked ? JoinStyle.FlatCut : JoinStyle.RoundFillet;
        var filleted = BezierMath.ApplyFilletForPath(new List<PathSegment> { seg1, seg2 }, r, style, out float effectiveRadius, closed: false);

        using var grayPen = new Pen(Color.LightGray, 2);
        using var bluePen = new Pen(Color.FromArgb(40, 90, 220), 3);
        using var helperPen = new Pen(Color.DarkGray, 1) { DashStyle = DashStyle.Dash };

        DrawPathSegment(e.Graphics, seg1, grayPen);
        DrawPathSegment(e.Graphics, seg2, grayPen);

        foreach (var s in filleted)
        {
            DrawPathSegment(e.Graphics, s, bluePen);
        }

        DrawControlPolygon(e.Graphics, helperPen, seg1);
        DrawControlPolygon(e.Graphics, helperPen, seg2);
        DrawControlPoints(e.Graphics);

        _info.Text = $"请求半径 r = {r}px, 有效半径 = {effectiveRadius:F2}px   输出段数 = {filleted.Count}   模式: {seg1.Kind} + {seg2.Kind} + {style}";
    }

    private static void DrawPathSegment(Graphics g, PathSegment s, Pen pen)
    {
        if (s.Kind == SegmentKind.Line)
        {
            g.DrawLine(pen, s.P0, s.P3);
        }
        else
        {
            g.DrawBezier(pen, s.P0, s.P1, s.P2, s.P3);
        }
    }

    private static void DrawControlPolygon(Graphics g, Pen pen, PathSegment s)
    {
        if (s.Kind == SegmentKind.Line)
        {
            g.DrawLine(pen, s.P0, s.P3);
        }
        else
        {
            g.DrawLines(pen, new[] { s.P0, s.P1, s.P2, s.P3 });
        }
    }

    private void DrawControlPoints(Graphics g)
    {
        for (int i = 0; i < _ctrl.Count; i++)
        {
            bool hiddenByLineMode = (_seg1Line.Checked && (i == 1 || i == 2)) || (_seg2Line.Checked && (i == 4 || i == 5));
            if (hiddenByLineMode) continue;

            RectangleF rc = new(_ctrl[i].X - 4, _ctrl[i].Y - 4, 8, 8);
            Brush b = i == 3 ? Brushes.OrangeRed : Brushes.Black;
            g.FillEllipse(b, rc);
        }
    }
}

internal static class Program
{
    [STAThread]
    private static void Main()
    {
        ApplicationConfiguration.Initialize();
        Application.Run(new MainForm());
    }
}
