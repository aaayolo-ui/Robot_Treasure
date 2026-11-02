const fs=require('fs'),vm=require('vm'),assert=require('assert');
const html=fs.readFileSync(__dirname+'/assets/laser_live.html','utf8');
const code=html.slice(html.indexOf('    const timing=data.round_timer'),html.indexOf("    document.querySelector('#drive-time').textContent"));
function duration(data){const ctx={data};vm.createContext(ctx);return vm.runInContext(code+'duration;',ctx);}
assert.equal(duration({round_timer_version:1,round_timer:{elapsed_ms:888888}}),null);
assert.equal(duration({round_timer_version:2,round_timer:{elapsed_ms:888888}}),null);
assert.equal(duration({round_timer_version:3,round_timer:{elapsed_ms:1234}}),1234);
assert.equal(duration({round_timer_version:3,round_timer:{elapsed_ms:null}}),null);
console.log('Old RESET counter is never displayed as a KEY2 round.');
