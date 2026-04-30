#!/usr/bin/env node

const http = require('http');
const fs = require('fs');
const path = require('path');

const publicDir = path.join(__dirname, 'public');
const parsedPort = Number.parseInt(process.env.PORT || '', 10);
const port = Number.isInteger(parsedPort) && parsedPort > 0 ? parsedPort : 3000;
const types = {
  '.html': 'text/html; charset=utf-8',
  '.css': 'text/css; charset=utf-8',
  '.js': 'text/javascript; charset=utf-8',
  '.json': 'application/json; charset=utf-8',
  '.svg': 'image/svg+xml; charset=utf-8'
};
const badRequest = Symbol('badRequest');

function safeFilePath(urlPath) {
  let decoded;
  try {
    decoded = decodeURIComponent(urlPath.split('?')[0]);
  } catch {
    return badRequest;
  }
  const normalized = path.normalize(decoded === '/' ? 'index.html' : decoded.replace(/^\/+/, ''));
  const filePath = path.join(publicDir, normalized);
  return filePath.startsWith(publicDir) ? filePath : null;
}

const server = http.createServer((req, res) => {
  const filePath = safeFilePath(req.url || '/');
  if (filePath === badRequest) {
    res.writeHead(400, { 'Content-Type': 'text/plain; charset=utf-8' });
    res.end('Bad request');
    return;
  }
  if (!filePath) {
    res.writeHead(403, { 'Content-Type': 'text/plain; charset=utf-8' });
    res.end('Forbidden');
    return;
  }

  fs.readFile(filePath, (error, data) => {
    if (error) {
      res.writeHead(404, { 'Content-Type': 'text/plain; charset=utf-8' });
      res.end('Not found');
      return;
    }

    res.writeHead(200, {
      'Content-Type': types[path.extname(filePath)] || 'application/octet-stream',
      'Cache-Control': 'no-store'
    });
    res.end(data);
  });
});

server.listen(port, () => {
  console.log(`Arcade cabinet running at http://localhost:${port}`);
});
