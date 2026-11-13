let preparationJob = null;
let preparationBundle = null;
function renderPreparation() {
  const card=$('requirements-preparation');if (!card) return;
  let output='';
  if (preparationJob) output+=`<p role="status">${esc(preparationJob.stage)}${preparationJob.error ? ' · '+esc(preparationJob.error) : ''}</p>`;
  if (preparationBundle) {
    const b=preparationBundle;
    output+=`<p>Geometry: <strong>${b.geometry_ready?'Ready':'Incomplete'}</strong> · ${b.collision_pairs} collision pairs.</p>${b.unmodelled_links.length ? `<p>Unresolved links: ${esc(b.unmodelled_links.join(', '))}</p>` : ''}<p class="small">Model ${esc(b.description_sha)} · reach map ${esc(b.reach_status)}.</p><button class="button" data-use-bundle ${!b.geometry_ready?'disabled':''}>Use this geometry for tests</button>`;
    if (b.fixtures_error) output+=`<p>Geometry is recorded; fixture generation was unavailable: ${esc(b.fixtures_error)}</p>`;
    if (b.fixtures) output+=`<h3>Simulator fixture candidates</h3><p>MuJoCo ${esc(b.fixtures.oracle_version)} labelled these offline. Review model alignment, collision masks and exclusions before accepting fixtures.</p>${Object.entries(b.fixtures.groups).map(([name,g])=>`<details class="fixture-section"><summary>${esc(name)} · ${g.candidates.length} candidates</summary>${g.error ? `<p>${esc(g.error)}</p>` : ''}<p class="small">${g.covered_links?.length || 0} matching robot links · ${g.ignored_contacts?.length || 0} excluded contact pairs · ${g.uncovered_pairs?.length || 0} pairs without simulator geometry.</p>${g.uncovered_pairs?.length ? '<p class="small">Clear poses cannot be verified until the simulator covers these pairs. Colliding candidates remain available for review.</p>' : ''}${g.candidates.map(c=>`<label class="command-toggle"><input type="checkbox" data-fixture-candidate="${esc(c.id)}" ${c.label==='unknown'?'disabled':''} />${esc({clear:'Clear pose',colliding:'Collision pose',unknown:'Unverified pose'}[c.label] || c.label)} · ${esc(c.id)}</label><p class="small">${c.expected_pairs.length ? esc(c.expected_pairs.map(p=>p.join(' ↔ ')).join(', ')) : 'No penetrations among aligned, enabled self-collision pairs.'}</p><details><summary>Joint positions</summary><pre class="technical">${esc(JSON.stringify(c.positions,null,2))}</pre></details>`).join('')}</details>`).join('')}<label class="command-toggle"><input type="checkbox" id="preparation-reviewed" />I reviewed simulator geometry, contact masks and collision exclusions</label><button class="button" data-use-fixtures>Add selected fixtures to draft settings</button><p class="small">This changes an editable draft. Save settings to use reviewed fixtures in a run. No collision pose is physically commanded.</p>`;
  }
  card.querySelector('.preparation-result').innerHTML=output;
}
function preparationCard() {
  return `<section class="panel settings-card" id="requirements-preparation"><div class="body"><h2>Prepare robot</h2><p>Capture collision geometry once and reuse it across runs. Optional MuJoCo sampling produces independent fixture candidates without moving the robot.</p><a class="textbutton" href="/robot-preparation.md" target="_blank" rel="noopener">Preparation guide</a><button class="button" data-find-bundles>Load a recorded bundle</button><div class="recorded-bundles"></div><button class="button" data-find-environments>Find running simulation environments</button><label class="field">ROS / simulation container<select id="preparation-environment"><option value="">Choose an environment</option></select></label><details data-open-key="preparation-environment"><summary>Environment settings</summary><label class="field">Docker container<input id="preparation-container" type="text" /></label><label class="field">MuJoCo model path in the container · optional<input id="preparation-model" type="text" /></label><label class="field">Python with MuJoCo<input id="preparation-python" type="text" value="/opt/mjviser/bin/python" /></label><label class="field">Maximum samples per group<input id="preparation-samples" type="number" min="1" max="1000" value="100" /></label></details><button class="button" data-prepare-robot>Prepare geometry and candidates</button><div class="preparation-result"></div></div></section>`;
}
async function pollPreparation(id) {
  preparationJob=await api('/api/preparation/'+id);preparationBundle=preparationJob.result;
  renderPreparation();
  if (preparationJob.status==='running') setTimeout(()=>pollPreparation(id).catch(e=>settingsNotice(e.message)),1500);
}
function installPreparationHandlers() {
  $('test-requirements').addEventListener('change',e=>{
    if (e.target.id==='preparation-environment' && e.target.value) {
      const value=JSON.parse(e.target.value);
      $('preparation-container').value=value.container;$('preparation-model').value=value.model;$('preparation-python').value=value.python;
    }
  });
  $('test-requirements').addEventListener('click',e=>{
    const button=e.target.closest('button');if (!button) return;
    if (![...button.attributes].some(a=>['data-find-environments','data-prepare-robot','data-use-bundle','data-use-fixtures','data-find-bundles','data-load-bundle'].includes(a.name))) return;
    settingsAction(async()=>{
      if (button.hasAttribute('data-find-bundles')) {
        const response=await api('/api/preparation/find',{uri:$('uri').value.trim(),profile:$('profile').value});
        $('requirements-preparation').querySelector('.recorded-bundles').innerHTML=response.bundles.length ? response.bundles.map(b=>`<button class="button" data-load-bundle="${esc(b.key)}">${esc(b.robot)} · ${esc(b.recorded)} · ${b.geometry_ready?'geometry ready':'incomplete'}</button>`).join('') : '<p>No recorded bundle matches this model.</p>';
      } else if (button.dataset.loadBundle) {
        const response=await api('/api/preparation/load',{uri:$('uri').value.trim(),profile:$('profile').value,key:button.dataset.loadBundle});
        await pollPreparation(response.id);
      } else if (button.hasAttribute('data-find-environments')) {
        const response=await api('/api/preparation/environments');
        $('preparation-environment').innerHTML='<option value="">Choose an environment</option>'+response.environments.map(env=>`<option value="${esc(JSON.stringify(env))}">${esc(env.container)}</option>`).join('');
        if (!response.environments.length) settingsNotice('No running MuJoCo environments found. Enter a ROS container under Environment settings.');
      } else if (button.hasAttribute('data-prepare-robot')) {
        button.disabled=true;
        try {
          const response=await api('/api/preparation/start',{uri:$('uri').value.trim(),profile:$('profile').value,container:$('preparation-container').value,model:$('preparation-model').value,python:$('preparation-python').value,samples:Number($('preparation-samples').value)});
          await pollPreparation(response.id);
        } finally {button.disabled=false;}
      } else if (button.hasAttribute('data-use-bundle')) {
        if (preparationBundle.description_sha!==settingsMeta.description_sha) throw Error('Bundle does not match the inspected model');
        reqSet(['artifacts','snapshot_dir'],preparationBundle.snapshot_root);renderRequirements();renderPreparation();
      } else if (button.hasAttribute('data-use-fixtures')) {
        if (!$('preparation-reviewed').checked) throw Error('Review simulator geometry, contact masks and exclusions before accepting fixtures');
        const selected=new Set([...$('requirements-preparation').querySelectorAll('[data-fixture-candidate]:checked')].map(el=>el.dataset.fixtureCandidate));
        if (!selected.size) throw Error('Select the clear or colliding candidates to use');
        if (preparationBundle.description_sha!==settingsMeta.description_sha) throw Error('Robot model changed; prepare it again');
        const reviewed=await api('/api/preparation/review',{uri:$('uri').value.trim(),profile:$('profile').value,job:preparationJob.id,reviewed:true,selected:[...selected]});
        for (const [name,recipe] of Object.entries(reviewed.groups)) {
          Object.assign((settingsDraft.groups ||= {})[name] ||= {},recipe);
        }
        reqSet(['artifacts','snapshot_dir'],preparationBundle.snapshot_root);
        settingsDraft.profile.description_sha=preparationBundle.description_sha;
        reqDirty();renderRequirements();renderPreparation();
      }
    });
  });
}
