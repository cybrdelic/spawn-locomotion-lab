import {defineConfig} from 'vite';
import {fileURLToPath} from 'node:url';
const entry=name=>fileURLToPath(new URL(name,import.meta.url));
export default defineConfig({build:{rolldownOptions:{input:{planar:entry('./index.html'),threeD:entry('./3d.html'),policy:entry('./policy.html')}}}});
