using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Management;
using System.Net;
using System.Reflection;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Web.Script.Serialization;
using System.Windows.Forms;

static class Backend
{
    public const string Api="http://127.0.0.1:8766";
    public const string PublicUrl="https://robot-treasure-monitor-1026.nicemug257.chatgpt.site/laser_live.html";
    public static readonly string Root=@"C:\Users\Administrator\Desktop\jiezhezuo\Robot_Treasure-main\Robot_Treasure-main\Tools\MapTimingLogger";
    public static readonly string DataRoot=Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData),"RobotTreasureMonitor");
    public static Dictionary<string,object> Request(string path,object payload=null)
    {
        var req=(HttpWebRequest)WebRequest.Create(Api+path);
        req.Proxy=null;req.Timeout=1500;req.ReadWriteTimeout=1500;
        if(payload!=null) {
            req.Method="POST";req.ContentType="application/json";
            byte[] bytes=Encoding.UTF8.GetBytes(new JavaScriptSerializer().Serialize(payload));
            req.ContentLength=bytes.Length;
            using(var stream=req.GetRequestStream())stream.Write(bytes,0,bytes.Length);
        }
        try {
            using(var response=req.GetResponse())
            using(var reader=new StreamReader(response.GetResponseStream(),Encoding.UTF8))
                return new JavaScriptSerializer().Deserialize<Dictionary<string,object>>(reader.ReadToEnd());
        } catch(WebException ex) {
            if(ex.Response!=null) {
                using(var reader=new StreamReader(ex.Response.GetResponseStream(),Encoding.UTF8)) {
                    var error=new JavaScriptSerializer().Deserialize<Dictionary<string,object>>(reader.ReadToEnd());
                    throw new InvalidOperationException(Value(error,"error"));
                }
            }
            throw new IOException("本地后台未响应，请先开启本地后台。",ex);
        }
    }
    public static bool Flag(Dictionary<string,object>d,string key)
    {return d.ContainsKey(key)&&d[key]is bool&&(bool)d[key];}
    public static string Value(Dictionary<string,object>d,string key)
    {return d.ContainsKey(key)&&d[key]!=null?d[key].ToString():"";}
    public static Dictionary<string,object> TryState()
    {try{return Request("/api/laser");}catch{return null;}}
    // Match the full path, not just a generic Python process name.
    public static List<int> ProcessesFor(string script)
    {
        string expected=Path.Combine(Root,script).Replace('/','\\').ToLowerInvariant();
        var ids=new List<int>();
        using(var search=new ManagementObjectSearcher("SELECT ProcessId,CommandLine FROM Win32_Process WHERE Name='python.exe' OR Name='pythonw.exe'"))
        using(var result=search.Get())foreach(ManagementObject item in result) {
            string cmd=Convert.ToString(item["CommandLine"]).Replace('/','\\').ToLowerInvariant();
            if(cmd.Contains("\""+expected+"\"")||cmd.Split(' ').Contains(expected))ids.Add(Convert.ToInt32(item["ProcessId"]));
        }
        return ids;
    }
    static Process Start(string script,string arguments)
    {
        string path=Path.Combine(Root,script);
        if(!File.Exists(path))throw new IOException("找不到监控程序，请保留桌面 jiezhezuo 文件夹。\n"+path);
        string python=@"E:\Python\pythonw.exe";
        if(!File.Exists(python))throw new IOException("找不到 E:\\Python\\pythonw.exe，请检查 Python 安装位置。");
        Directory.CreateDirectory(DataRoot);
        string host=Path.Combine(DataRoot,"desktop_monitor_host.py");
        using(var src=Assembly.GetExecutingAssembly().GetManifestResourceStream("desktop_monitor_host.py"))
        using(var dest=File.Create(host))src.CopyTo(dest);
        return Process.Start(new ProcessStartInfo(python,"-X utf8 \""+host+"\" \""+path+"\" "+arguments) {
            WorkingDirectory=Root,UseShellExecute=true,WindowStyle=ProcessWindowStyle.Hidden
        });
    }
    public static void StartLocal()
    {
        var state=TryState();
        if(state!=null) {
            if(!state.ContainsKey("serial_enabled"))throw new IOException("本地监控版本过旧，请关闭旧后台后重试。");
            return;
        }
        if(ProcessesFor("laser_live_server.py").Count>0)throw new IOException("现有后台没有响应，请先点“关闭后台”再开启。");
        using(var child=Start("laser_live_server.py","--port COM9 --http-port 8766")) {
            for(int i=0;i<20;i++) {
                Thread.Sleep(250);state=TryState();
                if(state!=null&&state.ContainsKey("serial_enabled"))return;
                if(child.HasExited)break;
            }
        }
        throw new IOException("后台启动失败，点“查看日志”检查详情。");
    }
    public static void StartCloud()
    {
        if(TryState()==null)throw new IOException("请先开启本地后台，再开启公网同步。");
        if(ProcessesFor("cloud_bridge.py").Count>0)return;
        if(!File.Exists(Path.Combine(DataRoot,"site.json")))throw new IOException("这台电脑尚未配置公网同步，请先使用本地监控。");
        using(var child=Start("cloud_bridge.py","")) {
            Thread.Sleep(900);
            if(child.HasExited)throw new IOException("公网同步启动失败，点“查看日志”检查详情。");
        }
    }
    public static void Stop(string script)
    {
        foreach(int id in ProcessesFor(script)) {
            try{using(var p=Process.GetProcessById(id)){p.Kill();if(!p.WaitForExit(2500))throw new IOException("后台未能退出，请重试。");}}
            catch(ArgumentException){}
        }
        if(ProcessesFor(script).Count>0)throw new IOException("后台仍在运行，请重试关闭。");
    }
    public static void StopLocal()
    {
        Stop("cloud_bridge.py");
        try{if(TryState()!=null)Request("/api/serial",new{action="disconnect"});}catch(IOException){}
        Stop("laser_live_server.py");
        if(TryState()!=null)throw new IOException("8766 仍被其他后台占用，未关闭不属于本工具的程序。");
    }
}

class Card:Panel
{
    public Card(){DoubleBuffered=true;BackColor=Color.White;}
    protected override void OnPaint(PaintEventArgs e){base.OnPaint(e);using(var pen=new Pen(Color.FromArgb(224,231,240)))e.Graphics.DrawRectangle(pen,0,0,Width-1,Height-1);}
}
class ActionButton:Button
{
    public ActionButton(string text,bool primary){
        Text=text;FlatStyle=FlatStyle.Flat;FlatAppearance.BorderSize=1;
        BackColor=primary?Color.FromArgb(44,104,225):Color.White;
        ForeColor=primary?Color.White:Color.FromArgb(49,67,92);
        FlatAppearance.BorderColor=primary?BackColor:Color.FromArgb(211,222,236);
        FlatAppearance.MouseOverBackColor=primary?Color.FromArgb(32,86,199):Color.FromArgb(239,245,255);
        Cursor=Cursors.Hand;UseVisualStyleBackColor=false;
    }
}
class MonitorSwitch:Form
{
    static readonly Color Ink=Color.FromArgb(31,49,74),Muted=Color.FromArgb(112,128,150);
    ComboBox ports=new ComboBox();
    Label localStatus,serialStatus,cloudStatus,feedback;
    ActionButton startLocal,stopLocal,connect,disconnect,startCloud,stopCloud,refresh,openLocal,openCloud,logs;
    System.Windows.Forms.Timer timer=new System.Windows.Forms.Timer();
    bool busy,polling,closing,localRunning,cloudRunning,serialEnabled,applyingPorts;
    string selectedPort="COM9";
    public MonitorSwitch()
    {
        Text="小车监控开关";ClientSize=new Size(720,638);
        Font=new Font("Microsoft YaHei UI",10F);ForeColor=Ink;BackColor=Color.FromArgb(244,247,252);
        AutoScaleMode=AutoScaleMode.Dpi;FormBorderStyle=FormBorderStyle.FixedSingle;MaximizeBox=false;
        StartPosition=FormStartPosition.CenterScreen;DoubleBuffered=true;
        Icon=Icon.ExtractAssociatedIcon(Application.ExecutablePath);
        LabelAt(this,"ROBOT TREASURE",26,17,400,21,9F,Muted,true);
        LabelAt(this,"小车监控开关",24,41,460,42,23F,Ink,true);
        LabelAt(this,"手动控制 · 随开随关 · 无开机自启",26,89,640,24,10F,Muted,false);
        var c1=AddCard(24,130,672,115);
        LabelAt(c1,"01   本地监控后台",18,14,350,25,12F,Ink,true);
        localStatus=LabelAt(c1,"正在读取状态…",18,48,360,24,10F,Muted,false);
        LabelAt(c1,"电脑接收数据与本地网页的服务",18,78,370,22,9F,Muted,false);
        startLocal=AddButton(c1,"开启后台",402,24,116,true);
        stopLocal=AddButton(c1,"关闭后台",534,24,116,false);
        var c2=AddCard(24,258,672,151);
        LabelAt(c2,"02   网页串口",18,14,300,25,12F,Ink,true);
        serialStatus=LabelAt(c2,"未连接",18,48,630,25,10F,Muted,false);
        ports.SetBounds(18,91,158,30);ports.DropDownStyle=ComboBoxStyle.DropDownList;c2.Controls.Add(ports);
        refresh=AddButton(c2,"刷新",190,87,72,false);
        connect=AddButton(c2,"连接串口",402,87,116,true);
        disconnect=AddButton(c2,"断开串口",534,87,116,false);
        var c3=AddCard(24,422,672,115);
        LabelAt(c3,"03   公网同步",18,14,350,25,12F,Ink,true);
        cloudStatus=LabelAt(c3,"正在读取状态…",18,48,380,24,10F,Muted,false);
        LabelAt(c3,"将本机收到的数据同步到公网监控",18,78,380,22,9F,Muted,false);
        startCloud=AddButton(c3,"开启同步",402,24,116,true);
        stopCloud=AddButton(c3,"关闭同步",534,24,116,false);
        openLocal=AddButton(this,"打开本地网页",24,553,214,false);
        openCloud=AddButton(this,"打开公网网页",251,553,214,false);
        logs=AddButton(this,"查看日志",478,553,218,false);
        feedback=LabelAt(this,"打开工具只查看状态；点击按钮后才开启服务。",26,607,668,23,9F,Muted,false);
        ports.SelectedIndexChanged+=(s,e)=>{if(!applyingPorts&&ports.SelectedItem!=null)selectedPort=ports.SelectedItem.ToString();};
        startLocal.Click+=async(s,e)=>await Run("正在开启本地后台…",async()=>{await Task.Run((Action)Backend.StartLocal);await Poll();await RefreshPorts();},"本地后台已开启。选择串口后点“连接串口”。");
        stopLocal.Click+=async(s,e)=>await Run("正在关闭本地后台…",()=>Task.Run((Action)Backend.StopLocal),"本地后台与公网同步已关闭，串口已释放。");
        refresh.Click+=async(s,e)=>await Run("正在刷新串口…",RefreshPorts,"串口列表已刷新。");
        connect.Click+=async(s,e)=>{
            if(ports.SelectedItem==null){feedback.Text="请插入 USB 串口，刷新后选择端口。";return;}
            string port=ports.SelectedItem.ToString();
            await Run("正在请求连接 "+port+"…",()=>Task.Run(()=>Backend.Request("/api/serial",new{action="connect",port=port})),"已提交连接请求，请查看串口状态。");
        };
        disconnect.Click+=async(s,e)=>await Run("正在释放串口…",()=>Task.Run(()=>Backend.Request("/api/serial",new{action="disconnect"})),"串口已断开，可以打开串口助手。");
        startCloud.Click+=async(s,e)=>await Run("正在开启公网同步…",()=>Task.Run((Action)Backend.StartCloud),"同步进程已开启；网页是否可访问取决于网络状态。");
        stopCloud.Click+=async(s,e)=>await Run("正在关闭公网同步…",()=>Task.Run(()=>Backend.Stop("cloud_bridge.py")),"公网同步已关闭，本地接收保持原状态。");
        openLocal.Click+=(s,e)=>Open(Backend.Api+"/laser_live.html");
        openCloud.Click+=(s,e)=>Open(Backend.PublicUrl);
        logs.Click+=(s,e)=>{Directory.CreateDirectory(Path.Combine(Backend.DataRoot,"logs"));Open(Path.Combine(Backend.DataRoot,"logs"));};
        timer.Interval=2500;timer.Tick+=async(s,e)=>{if(!busy)await Poll();};
        Shown+=async(s,e)=>{await Poll();await RefreshPorts();timer.Start();};
        FormClosing+=OnClosing;SetButtons();
    }
    Label LabelAt(Control parent,string text,int x,int y,int w,int h,float size,Color color,bool bold){
        var label=new Label{Text=text,Location=new Point(x,y),Size=new Size(w,h),ForeColor=color,
            Font=new Font(Font.FontFamily,size,bold?FontStyle.Bold:FontStyle.Regular),AutoEllipsis=true};
        parent.Controls.Add(label);return label;
    }
    Card AddCard(int x,int y,int w,int h){var card=new Card();card.SetBounds(x,y,w,h);Controls.Add(card);return card;}
    ActionButton AddButton(Control parent,string text,int x,int y,int w,bool primary){
        var button=new ActionButton(text,primary);button.SetBounds(x,y,w,40);parent.Controls.Add(button);return button;
    }
    void Open(string target){try{Process.Start(new ProcessStartInfo(target){UseShellExecute=true});}catch(Exception ex){feedback.Text=ex.Message;}}
    async Task RefreshPorts(){
        try{
            string[] list;
            if(localRunning){
                var data=await Task.Run(()=>Backend.Request("/api/ports"));
                list=((ArrayList)data["ports"]).Cast<object>().Select(x=>x.ToString()).ToArray();
                if(selectedPort==null)selectedPort=Backend.Value(data,"selected");
            }else list=await Task.Run(()=>System.IO.Ports.SerialPort.GetPortNames().OrderBy(x=>x).ToArray());
            if(IsDisposed)return;
            applyingPorts=true;ports.Items.Clear();ports.Items.AddRange(list);
            if(selectedPort!=null&&ports.Items.Contains(selectedPort))ports.SelectedItem=selectedPort;
            else if(ports.Items.Count>0)ports.SelectedIndex=0;
            applyingPorts=false;
            if(ports.SelectedItem!=null)selectedPort=ports.SelectedItem.ToString();
        }catch(Exception ex){if(!IsDisposed)feedback.Text="刷新失败："+ex.Message;}
    }
    async Task Poll(){
        if(polling||IsDisposed)return;polling=true;
        try{
            var snapshot=await Task.Run(()=>new{State=Backend.TryState(),Local=Backend.ProcessesFor("laser_live_server.py").Count>0,Cloud=Backend.ProcessesFor("cloud_bridge.py").Count>0});
            if(IsDisposed)return;
            localRunning=snapshot.State!=null;cloudRunning=snapshot.Cloud;
            localStatus.Text=localRunning?"● 已开启 · 本地网页可用":snapshot.Local?"● 进程存在，但未响应；可关闭后重开":"○ 已关闭";
            localStatus.ForeColor=localRunning?Color.FromArgb(28,133,99):Muted;
            serialEnabled=localRunning&&Backend.Flag(snapshot.State,"serial_enabled");
            bool connected=localRunning&&Backend.Flag(snapshot.State,"connected");
            serialStatus.Text=localRunning?Backend.Value(snapshot.State,"port")+" · "+(connected?"已连接 · 115200 / 8N1":serialEnabled?"连接中 / 等待串口":"已断开 · 可使用串口助手"):"本地后台已关闭 · 串口未被本工具占用";
            serialStatus.ForeColor=connected?Color.FromArgb(28,133,99):Muted;
            if(serialEnabled&&!connected&&Backend.Value(snapshot.State,"error").Length>0)serialStatus.Text=Backend.Value(snapshot.State,"error");
            cloudStatus.Text=cloudRunning?"● 同步进程运行中 · 约 1 秒上传一次":"○ 已关闭 · 公网网页稍后显示离线";
            cloudStatus.ForeColor=cloudRunning?Color.FromArgb(28,133,99):Muted;
        }catch(Exception ex){if(!IsDisposed)feedback.Text="状态读取失败："+ex.Message;}
        finally{polling=false;if(!IsDisposed)SetButtons();}
    }
    void SetButtons(){
        startLocal.Enabled=!busy&&!localRunning;stopLocal.Enabled=!busy;
        connect.Enabled=!busy&&localRunning&&!serialEnabled;disconnect.Enabled=!busy&&localRunning;
        startCloud.Enabled=!busy&&localRunning&&!cloudRunning;stopCloud.Enabled=!busy&&cloudRunning;
        refresh.Enabled=!busy;ports.Enabled=!busy&&!serialEnabled;
        openLocal.Enabled=!busy&&localRunning;openCloud.Enabled=!busy;logs.Enabled=!busy;
    }
    async Task Run(string pending,Func<Task>action,string success){
        if(busy)return;busy=true;SetButtons();feedback.Text=pending;
        try{await action();if(!IsDisposed)feedback.Text=success;}
        catch(Exception ex){if(!IsDisposed){feedback.Text="操作未完成："+ex.Message;MessageBox.Show(this,ex.Message,"操作未完成",MessageBoxButtons.OK,MessageBoxIcon.Information);}}
        finally{busy=false;if(!IsDisposed)SetButtons();}
        if(!IsDisposed)await Poll();
    }
    async void OnClosing(object sender,FormClosingEventArgs e){
        if(closing)return;e.Cancel=true;if(busy)return;
        if(localRunning||cloudRunning){
            var result=MessageBox.Show(this,"是否同时关闭后台和公网同步，并释放串口？\n\n是：全部关闭。\n否：仅关闭窗口，后台继续运行。",Text,MessageBoxButtons.YesNoCancel,MessageBoxIcon.Question);
            if(result==DialogResult.Cancel)return;
            if(result==DialogResult.Yes){
                bool stopped=false;
                await Run("正在关闭全部服务…",async()=>{await Task.Run((Action)Backend.StopLocal);stopped=true;},"全部服务已关闭。");
                if(!stopped)return;
            }
        }
        closing=true;timer.Stop();Close();
    }
    [STAThread]
    static void Main(string[]args){
        Application.EnableVisualStyles();Application.SetCompatibleTextRenderingDefault(false);
        if(args.Length==2&&args[0]=="--verify-lifecycle"){
            var checks=new List<string>();
            try{
                Backend.StopLocal();checks.Add("stop_existing");Backend.StartLocal();checks.Add("manual_start");
                var state=Backend.Request("/api/laser");
                if(Backend.Flag(state,"connected")||Backend.Flag(state,"serial_enabled"))throw new Exception("Serial opened unexpectedly");
                checks.Add("default_disconnected");Backend.Request("/api/ports");checks.Add("enumerate_ports");
                Backend.StartCloud();checks.Add("manual_start_cloud");
                Backend.Stop("cloud_bridge.py");checks.Add("manual_stop_cloud");
                if(Backend.TryState()==null)throw new Exception("Stopping cloud stopped local service");
                checks.Add("cloud_stop_preserves_local");
                Backend.Request("/api/serial",new{action="disconnect"});checks.Add("disconnect_api");Backend.StopLocal();checks.Add("manual_stop");
                File.WriteAllText(args[1],new JavaScriptSerializer().Serialize(new{ok=true,checks=checks}),Encoding.UTF8);
            }catch(Exception ex){File.WriteAllText(args[1],new JavaScriptSerializer().Serialize(new{ok=false,error=ex.ToString(),checks=checks}),Encoding.UTF8);Environment.ExitCode=1;}
            return;
        }
        bool created;
        using(var mutex=new Mutex(true,@"Local\RobotTreasureSerialSwitchV2",out created)){
            if(!created){MessageBox.Show("小车监控开关已经打开，请查看任务栏。","小车监控开关");return;}
            Application.Run(new MonitorSwitch());
        }
    }
}
