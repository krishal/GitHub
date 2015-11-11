// ==UserScript==
// @name       Force HTML 5 Video Gfycat
// @namespace  http://reddit.com/
// @version    1.0
// @description Converts giant.gfycat.com hyperlinks to their HTML 5 video counterpart.
// @match      https://www.reddit.com/*
// @copyright  2014
// @grant metadata
// ==/UserScript==

document.addEventListener('DOMContentLoaded', stripCats, false);
if (document.readyState === 'complete') {
  stripCats();
}

function stripCats() {
  
  Array.forEach(document.links, function (a) {
    a.href = a.href.replace(/giant\.(.*)\.gif/i, '$1');
  });

  Array.forEach(document.links, function (a) {
    a.href = a.href.replace('.gif', '.gifv');
    a.href = a.href.replace('.gifvv', '.gifv');
  });
}
