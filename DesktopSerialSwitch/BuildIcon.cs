using System; using System.Drawing; using System.Drawing.Drawing2D; using System.IO;
class BuildIcon {
 static void Main(string[] args) {
  using(var bitmap=new Bitmap(64,64)) using(var g=Graphics.FromImage(bitmap)) {
   g.SmoothingMode=SmoothingMode.AntiAlias;g.Clear(Color.Transparent);
   using(var path=new GraphicsPath()) {
    path.AddArc(1,1,22,22,180,90);path.AddArc(41,1,22,22,270,90);path.AddArc(41,41,22,22,0,90);path.AddArc(1,41,22,22,90,90);path.CloseFigure();
    using(var brush=new LinearGradientBrush(new Point(0,0),new Point(64,64),Color.FromArgb(74,137,248),Color.FromArgb(33,83,192)))g.FillPath(brush,path);
   }
   using(var pen=new Pen(Color.White,3.6f)) {g.DrawRectangle(pen,15,22,34,26);g.DrawLine(pen,32,15,32,21);g.DrawLine(pen,10,29,10,40);g.DrawLine(pen,54,29,54,40);g.DrawLine(pen,24,54,40,54);}
   g.FillEllipse(Brushes.White,28,9,8,8);g.FillEllipse(Brushes.White,23,30,5,5);g.FillEllipse(Brushes.White,37,30,5,5);
   using(var pen=new Pen(Color.White,2.8f))g.DrawLine(pen,27,40,37,40);
   using(var icon=Icon.FromHandle(bitmap.GetHicon()))using(var file=File.Create(args[0]))icon.Save(file);
  }
 }
}
