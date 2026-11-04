import http from 'node:http';
import {readFile} from 'node:fs/promises';
import {resolve, extname, sep, dirname} from 'node:path';
import {fileURLToPath} from 'node:url';

const root=dirname(fileURLToPath(import.meta.url));
const types={'.html':'text/html; charset=utf-8','.css':'text/css; charset=utf-8','.js':'text/javascript; charset=utf-8','.png':'image/png'};
http.createServer(async(req,res)=>{
 try{
  if(req.method!=='GET'&&req.method!=='HEAD'){res.writeHead(405,{Allow:'GET, HEAD'});res.end();return}
  const url=new URL(req.url,'http://localhost'),pathname=decodeURIComponent(url.pathname==='/'?'/index.html':url.pathname);
  // Only browser assets are served; project sources and dotfiles stay private.
  if(!/^\/(?:index\.html|style\.css|src\/[a-z0-9-]+\.js|references\/[a-z0-9._-]+\.png)$/i.test(pathname))throw Error('not found');
  const path=resolve(root,'.'+pathname);if(!path.startsWith(root+sep))throw Error('not found');
  const data=await readFile(path);res.writeHead(200,{'Content-Type':types[extname(path).toLowerCase()]||'application/octet-stream','X-Content-Type-Options':'nosniff'});res.end(req.method==='HEAD'?undefined:data);
 }catch{res.writeHead(404);res.end('Not found')}
}).listen(Number(process.env.PORT||4173),'127.0.0.1',()=>console.log('Local: http://127.0.0.1:'+(process.env.PORT||4173)+'/'));
