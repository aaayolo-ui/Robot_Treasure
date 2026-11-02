using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.IO;
using System.IO.Compression;
using System.Linq;
using System.Management;
using System.Net;
using System.Reflection;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Web.Script.Serialization;
using System.Windows.Forms;
using Microsoft.Web.WebView2.Core;
using Microsoft.Web.WebView2.WinForms;

static class Service
{
    public static readonly string Root=Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData),@"RobotTreasureMonitor\Integrated-v1");
    public const string Local="http://127.0.0.1:8766";
    public const string Public="https://robot-treasure-monitor-1026.nicemug257.chatgpt.site/laser_live.html";
    public static Dictionary<string,object> Call(string path,object payload=null,bool auth=false) {
        var req=(HttpWebRequest)WebRequest.Create(Local+path);req.Proxy=null;req.Timeout=2000;req.ReadWriteTimeout=2000;
        if(auth)req.Headers["X-Desktop-Key"]=File.ReadAllText(Path.Combine(Root,"console.key")).Trim();
        if(payload!=null){req.Method="POST";req.ContentType="application/json";byte[] b=Encoding.UTF8.GetBytes(new JavaScriptSerializer().Serialize(payload));req.ContentLength=b.Length;using(var s=req.GetRequestStream())s.Write(b,0,b.Length);}
        try{using(var r=req.GetResponse())using(var s=new StreamReader(r.GetResponseStream(),Encoding.UTF8))return new JavaScriptSerializer{MaxJsonLength=2000000}.Deserialize<Dictionary<string,object>>(s.ReadToEnd());}
        catch(WebException ex){if(ex.Response!=null){using(var s=new StreamReader(ex.Response.GetResponseStream(),Encoding.UTF8)){var d=new JavaScriptSerializer().Deserialize<Dictionary<string,object>>(s.ReadToEnd());throw new IOException(Convert.ToString(d["error"]));}}throw new IOException("后台未响应，请先开启后台。",ex);}
    }
    public static Dictionary<string,object> State(){try{return Call("/api/laser");}catch{return null;}}
    public static bool Flag(Dictionary<string,object>d,string key){return d!=null&&d.ContainsKey(key)&&d[key]is bool&&(bool)d[key];}
    public static List<int> Pids(string script) {
        string file=Path.Combine(Root,script).Replace('/','\\').ToLowerInvariant();var ids=new List<int>();
        using(var query=new ManagementObjectSearcher("SELECT ProcessId,CommandLine FROM Win32_Process WHERE Name='python.exe' OR Name='pythonw.exe'"))
        using(var rows=query.Get())foreach(ManagementObject r in rows){string line=Convert.ToString(r["CommandLine"]).Replace('/','\\').ToLowerInvariant();if(line.Contains("\""+file+"\"")||line.Split(' ').Contains(file))ids.Add(Convert.ToInt32(r["ProcessId"]));}
        return ids;
    }
    static Process Start(string script){
        string python=@"E:\Python\pythonw.exe";if(!File.Exists(python))throw new IOException("找不到 E:\\Python\\pythonw.exe，请检查本机 Python。");
        return Process.Start(new ProcessStartInfo(python,"-X utf8 \""+Path.Combine(Root,"host.py")+"\" \""+Path.Combine(Root,script)+"\""){UseShellExecute=true,WindowStyle=ProcessWindowStyle.Hidden,WorkingDirectory=Root});
    }
    public static void StartLocal(){
        var d=State();if(d!=null){if(!d.ContainsKey("console_version"))throw new IOException("旧监控后台正在运行。请先在旧的“小车监控开关”里关闭后台，再开启新版。");return;}
        if(Pids("console_server.py").Count>0)throw new IOException("后台没有响应，请先关闭后台再重开。");
        using(var p=Start("console_server.py")){for(int i=0;i<15;i++){Thread.Sleep(250);d=State();if(d!=null&&d.ContainsKey("console_version"))return;if(p.HasExited)break;}}
        throw new IOException("后台启动失败，请打开日志检查详情。");
    }
    public static void Stop(string script){foreach(int id in Pids(script))try{using(var p=Process.GetProcessById(id)){p.Kill();if(!p.WaitForExit(2500))throw new IOException("进程未退出，请重试。");}}catch(ArgumentException){} }
    public static void StopLocal(){Stop("cloud_bridge.py");try{if(State()!=null)Call("/api/serial",new{action="disconnect"});}catch{}Stop("console_server.py");}
    public static void StartCloud(){
        if(!Flag(State(),"connected"))throw new IOException("请先连接串口，再开启公网同步。");
        if(Pids("cloud_bridge.py").Count>0)return;
        string config=Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData),@"RobotTreasureMonitor\site.json");
        if(!File.Exists(config))throw new IOException("这台电脑尚未配置公网同步，请使用本地监控。");
        using(var p=Start("cloud_bridge.py")){Thread.Sleep(700);if(p.HasExited)throw new IOException("同步进程启动失败，请查看日志。");}
    }
}

class MainWindow:Form
{
    WebView2 ui=new WebView2(),site=new WebView2();
    System.Windows.Forms.Timer statusTimer=new System.Windows.Forms.Timer(),dataTimer=new System.Windows.Forms.Timer();
    SemaphoreSlim gate=new SemaphoreSlim(1,1);
    bool ready,polling,reading,closing,online;long offset;string stream="",view="serial";
    CoreWebView2Environment environment;
    public MainWindow(){
        Text="小车串口监控一体助手";ClientSize=new Size(1480,950);WindowState=FormWindowState.Maximized;MinimumSize=new Size(1050,720);StartPosition=FormStartPosition.CenterScreen;
        BackColor=Color.FromArgb(247,248,250);Icon=Icon.ExtractAssociatedIcon(Application.ExecutablePath);
        ui.Dock=DockStyle.Fill;Controls.Add(ui);site.Visible=false;Controls.Add(site);
        Shown+=async(s,e)=>{try{await Initialize();}catch(Exception ex){MessageBox.Show(this,ex.Message,"界面启动失败");}};
        statusTimer.Interval=1800;statusTimer.Tick+=async(s,e)=>await Poll();
        dataTimer.Interval=180;dataTimer.Tick+=async(s,e)=>await ReadData();
        FormClosing+=OnFormClosingRequested;
    }
    async Task Initialize(){
        environment=await CoreWebView2Environment.CreateAsync(null,Path.Combine(Service.Root,"browser"));
        await ui.EnsureCoreWebView2Async(environment);
        // Serve this application's three UI resources directly, so the browser
        // does not need filesystem access to the private application-data folder.
        ui.CoreWebView2.AddWebResourceRequestedFilter("https://app.robotmonitor.local/*",CoreWebView2WebResourceContext.All);
        ui.CoreWebView2.WebResourceRequested+=(s,e)=>{
            string path=new Uri(e.Request.Uri).AbsolutePath;
            string name=path=="/index.html"?"index.html":path=="/style.css"?"style.css":path=="/app.js"?"app.js":null;
            string type=path=="/index.html"?"text/html":path=="/style.css"?"text/css":"text/javascript";
            var bytes=name==null?new byte[0]:File.ReadAllBytes(Path.Combine(Service.Root,name));
            e.Response=environment.CreateWebResourceResponse(new MemoryStream(bytes),name==null?404:200,name==null?"Not Found":"OK","Content-Type: "+type+"; charset=utf-8\r\nCache-Control: no-store");
        };
        ui.CoreWebView2.Settings.AreDevToolsEnabled=false;
        ui.CoreWebView2.WebMessageReceived+=Message;
        ui.CoreWebView2.Navigate("https://app.robotmonitor.local/index.html");
    }
    void Post(object data){if(ready&&!IsDisposed)ui.CoreWebView2.PostWebMessageAsJson(new JavaScriptSerializer{MaxJsonLength=3000000}.Serialize(data));}
    async Task Poll(){
        if(!ready||polling||IsDisposed)return;polling=true;
        try{
            var packet=await Task.Run(()=>{
                var d=Service.State();bool compatible=d!=null&&d.ContainsKey("console_version");
                return new{type="state",running=compatible,legacy=d!=null&&!compatible,cloud=Service.Pids("cloud_bridge.py").Count>0,state=compatible?d:null};
            });
            online=packet.running;if(!IsDisposed)Post(packet);
        }catch(Exception ex){Post(new{type="notice",error=true,text=ex.Message});}finally{polling=false;}
    }
    async Task ReadData(){
        if(!ready||!online||reading||IsDisposed)return;reading=true;
        try{
            var result=await Task.Run(()=>Service.Call("/api/console?after="+offset,null,true));
            string current=Convert.ToString(result["stream"]);
            if(current!=stream){stream=current;offset=0;result=await Task.Run(()=>Service.Call("/api/console?after=0",null,true));}
            offset=Convert.ToInt64(result["next"]);Post(new{type="data",packet=result});
        }catch{}finally{reading=false;}
    }
    async void Message(object sender,CoreWebView2WebMessageReceivedEventArgs args){
        if(!args.Source.StartsWith("https://app.robotmonitor.local/",StringComparison.Ordinal))return;
        Dictionary<string,object> msg;
        try{msg=new JavaScriptSerializer{MaxJsonLength=3000000}.Deserialize<Dictionary<string,object>>(args.WebMessageAsJson);}catch{return;}
        string action=Convert.ToString(msg["action"]),id=msg.ContainsKey("id")?Convert.ToString(msg["id"]):"";
        if(action=="ready"){ready=true;statusTimer.Start();dataTimer.Start();await Ports();await Poll();return;}
        if(action=="layout"){
            double scale=Convert.ToDouble(msg["scale"]);site.SetBounds((int)(Convert.ToDouble(msg["x"])*scale),(int)(Convert.ToDouble(msg["y"])*scale),(int)(Convert.ToDouble(msg["w"])*scale),(int)(Convert.ToDouble(msg["h"])*scale));return;
        }
        await gate.WaitAsync();
        try{
            switch(action){
                case "start":await Task.Run((Action)Service.StartLocal);await Ports();break;
                case "stop":await Task.Run((Action)Service.StopLocal);site.Visible=false;view="serial";break;
                case "ports":await Ports();break;
                case "connect":await Task.Run(()=>Service.Call("/api/console/connect",msg["config"],true));break;
                case "disconnect":await Task.Run(()=>Service.Call("/api/serial",new{action="disconnect"}));break;
                case "cloud_on":await Task.Run((Action)Service.StartCloud);break;
                case "cloud_off":await Task.Run(()=>Service.Stop("cloud_bridge.py"));break;
                case "send":await Task.Run(()=>Service.Call("/api/console/send",new{data=msg["data"]},true));break;
                case "show":await ShowSite(Convert.ToString(msg["page"]));break;
                case "reload":if(site.CoreWebView2!=null)site.CoreWebView2.Reload();break;
                case "external":Process.Start(new ProcessStartInfo(view=="public"?Service.Public:Service.Local+"/laser_live.html"){UseShellExecute=true});break;
                case "logs":Directory.CreateDirectory(Path.Combine(Service.Root,"logs"));Process.Start(new ProcessStartInfo(Path.Combine(Service.Root,"logs")){UseShellExecute=true});break;
                case "save":using(var save=new SaveFileDialog{FileName="小车串口_"+DateTime.Now.ToString("yyyyMMdd_HHmmss")+".txt",Filter="文本文件|*.txt"}){if(save.ShowDialog(this)==DialogResult.OK)File.WriteAllText(save.FileName,Convert.ToString(msg["text"]),new UTF8Encoding(false));}break;
            }
            Post(new{type="result",id=id,ok=true,action=action});
        }catch(Exception ex){Post(new{type="result",id=id,ok=false,error=ex.Message,action=action});}
        finally{gate.Release();}
        await Poll();
    }
    async Task Ports(){var list=await Task.Run(()=>System.IO.Ports.SerialPort.GetPortNames().OrderBy(x=>x).ToArray());Post(new{type="ports",ports=list});}
    async Task ShowSite(string page){
        if(page=="serial"){site.Visible=false;view=page;return;}
        var state=await Task.Run((Func<Dictionary<string,object>>)Service.State);
        if(state==null||!state.ContainsKey("console_version"))throw new IOException("请先开启新版后台。");
        if(page=="public"&&Service.Pids("cloud_bridge.py").Count==0)throw new IOException("请先连接串口并开启公网同步。");
        if(page!="local"&&page!="public")return;
        if(site.CoreWebView2==null){
            await site.EnsureCoreWebView2Async(environment);
            // Keep display updates independent of the already-running serial
            // service; all /api requests still go to that single serial owner.
            site.CoreWebView2.AddWebResourceRequestedFilter(Service.Local+"/laser_live.html*",CoreWebView2WebResourceContext.All);
            site.CoreWebView2.AddWebResourceRequestedFilter(Service.Local+"/embedded.css*",CoreWebView2WebResourceContext.All);
            site.CoreWebView2.WebResourceRequested+=(s,e)=>{
                string path=new Uri(e.Request.Uri).AbsolutePath;
                string file=path=="/laser_live.html"?"laser_live.html":path=="/embedded.css"?"embedded.css":null;
                if(file==null)return;
                var bytes=File.ReadAllBytes(Path.Combine(Service.Root,file));
                e.Response=environment.CreateWebResourceResponse(new MemoryStream(bytes),200,"OK","Content-Type: "+(file.EndsWith(".css")?"text/css":"text/html")+"; charset=utf-8\r\nCache-Control: no-store");
            };
            site.CoreWebView2.NewWindowRequested+=(s,e)=>{e.Handled=true;site.CoreWebView2.Navigate(e.Uri);};
        }
        string url=page=="local"?Service.Local+"/laser_live.html":Service.Public;
        if(view!=page||site.Source==null)site.CoreWebView2.Navigate(url);
        view=page;site.Visible=true;site.BringToFront();
    }
    async void OnFormClosingRequested(object sender,FormClosingEventArgs e){
        if(closing)return;e.Cancel=true;
        if(gate.CurrentCount==0)return;
        bool active=await Task.Run(()=>Service.Pids("console_server.py").Count>0||Service.Pids("cloud_bridge.py").Count>0);
        if(active){var answer=MessageBox.Show(this,"退出时同时关闭后台、公网同步并释放串口？\n\n是：全部关闭。\n否：只关闭窗口，后台继续运行。",Text,MessageBoxButtons.YesNoCancel,MessageBoxIcon.Question);
            if(answer==DialogResult.Cancel)return;
            if(answer==DialogResult.Yes){try{await Task.Run((Action)Service.StopLocal);}catch(Exception ex){MessageBox.Show(this,ex.Message);return;}}}
        closing=true;statusTimer.Stop();dataTimer.Stop();Close();
    }
}

static class Bootstrap
{
    [DllImport("kernel32.dll",CharSet=CharSet.Unicode)]static extern bool SetDllDirectory(string path);
    [STAThread]static void Main(string[] args){
        try{
            Directory.CreateDirectory(Service.Root);
            using(var resource=Assembly.GetExecutingAssembly().GetManifestResourceStream("assets.zip"))
            using(var zip=new ZipArchive(resource,ZipArchiveMode.Read))foreach(var entry in zip.Entries){
                string destination=Path.GetFullPath(Path.Combine(Service.Root,entry.FullName));
                if(!destination.StartsWith(Service.Root+Path.DirectorySeparatorChar,StringComparison.OrdinalIgnoreCase))throw new IOException("资源路径无效");
                Directory.CreateDirectory(Path.GetDirectoryName(destination));
                using(var src=entry.Open())using(var memory=new MemoryStream()){
                    src.CopyTo(memory);byte[] bytes=memory.ToArray();
                    if(!File.Exists(destination)||!File.ReadAllBytes(destination).SequenceEqual(bytes))File.WriteAllBytes(destination,bytes);
                }
            }
            AppDomain.CurrentDomain.AssemblyResolve+=(s,e)=>{string path=Path.Combine(Service.Root,new AssemblyName(e.Name).Name+".dll");return File.Exists(path)?Assembly.LoadFrom(path):null;};
            SetDllDirectory(Service.Root);
            if(args.Length==2&&args[0]=="--verify"){
                var checks=new List<string>();
                try{Service.StopLocal();Service.StartLocal();checks.Add("start");var d=Service.State();if(Service.Flag(d,"connected")||Service.Flag(d,"serial_enabled"))throw new Exception("Unexpected serial connection");checks.Add("default_disconnected");Service.Call("/api/console?after=0",null,true);checks.Add("console_api");Service.Call("/api/ports");checks.Add("ports");Service.Call("/api/serial",new{action="disconnect"});checks.Add("disconnect");Service.StopLocal();checks.Add("stop");File.WriteAllText(args[1],new JavaScriptSerializer().Serialize(new{ok=true,checks=checks}));}
                catch(Exception ex){File.WriteAllText(args[1],new JavaScriptSerializer().Serialize(new{ok=false,error=ex.ToString(),checks=checks}));Environment.ExitCode=1;}return;
            }
            bool created;using(var mutex=new Mutex(true,@"Local\RobotTreasureIntegrated",out created)){if(!created){MessageBox.Show("一体助手已经打开，请查看任务栏。");return;}Run();}
        }catch(Exception ex){MessageBox.Show(ex.Message,"一体助手启动失败");}
    }
    [MethodImpl(MethodImplOptions.NoInlining)]static void Run(){Application.EnableVisualStyles();Application.SetCompatibleTextRenderingDefault(false);Application.Run(new MainWindow());}
}
