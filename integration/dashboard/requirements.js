/* Forms for robot-specific test prerequisites. Capturing state never commands motion. */
function reqField(label, path, value, type = "number", optional = false) {
  return `<label class="field">${esc(label)}<input data-requirement-path="${esc(JSON.stringify(path))}" data-optional="${optional}" type="${type}" ${type === "number" ? 'step="any"' : ""} value="${esc(value ?? "")}" /></label>`;
}
function reqToggle(label, path, checked) {
  return `<label class="command-toggle"><input type="checkbox" data-requirement-path="${esc(JSON.stringify(path))}" ${checked ? "checked" : ""} />${esc(label)}</label>`;
}
function reqCommand(label, path, value) {
  return `<label class="field">${esc(label)} · one argument per line<textarea class="fixture-command" data-requirement-path="${esc(JSON.stringify(path))}" data-argument-list="true" rows="3" spellcheck="false">${esc((value || []).join("\n"))}</textarea></label>`;
}
function reqDirty() {
  $("settings-save").disabled = false;
  settingsNotice("Unsaved changes");
}
function reqSet(path, value, remove = false) {
  let node = settingsDraft;
  for (let i = 0; i < path.length - 1; i++) node = node[path[i]] ||= typeof path[i + 1] === "number" ? [] : {};
  if (remove) delete node[path.at(-1)]; else node[path.at(-1)] = value;
  if (path[0]==='groups' && ['clear_configurations','collision_configurations','clear_state','collision_state'].includes(path[2])) delete settingsDraft.groups[path[1]].fixture_provenance;
  reqDirty();
}
function groupRequirementId(name) { return "requirements-group-" + encodeURIComponent(name); }
function poseRequirements(group, field, title) {
  const poses = settingsDraft.groups?.[group.name]?.[field] || [];
  const readings = groupReadings[group.name];
  const table = poses.length ? `<div class="fixture-table-wrap"><table class="fixture-table"><thead><tr><th>Joint</th><th>Units / limits</th>${poses.map((p, i) => `<th>Pose ${i + 1}<br><button class="textbutton" data-remove-pose="${i}" data-group="${esc(group.name)}" data-pose-field="${field}">Remove</button></th>`).join("")}</tr></thead><tbody>${group.joints.map((joint, index) => {
    const state = readings?.joints.find((j) => j.name === joint);
    const units = state ? `${state.unit}${state.limits ? ` · ${state.limits[0].toFixed(3)} to ${state.limits[1].toFixed(3)}` : ""}` : "Read state for limits";
    return `<tr><td>${esc(joint)}</td><td>${esc(units)}</td>${poses.map((p, i) => `<td><input type="number" step="any" aria-label="${esc(joint)} pose ${i + 1}" data-requirement-path="${esc(JSON.stringify(['groups',group.name,field,i,joint]))}" data-optional="true" value="${esc(p[joint] ?? '')}" ${state?.limits ? `min="${state.limits[0]}" max="${state.limits[1]}"` : ""} /></td>`).join("")}</tr>`;
  }).join("")}</tbody></table></div>` : '<p class="small">No poses configured.</p>';
  return `<details class="fixture-section" data-open-key="${esc(group.name + field)}"><summary>${title} · ${poses.length} poses</summary><p class="small">Enter targets by joint name. Capture fills measured positions; the first trajectory capture creates two targets. Edit one to exercise actual movement.</p>${table}<div class="row fixture-actions"><button class="button" data-add-pose="${field}" data-group="${esc(group.name)}">Add pose</button><button class="button" data-capture-pose="${field}" data-group="${esc(group.name)}">Capture current positions</button></div></details>`;
}
function stateRequirement(group, field, title) {
  const values = settingsDraft.groups?.[group.name]?.[field] || {};
  return `<details class="fixture-section" data-open-key="${esc(group.name + field)}"><summary>${title} · ${Object.keys(values).length} joints</summary><p class="small">Full measured state supplies independently moving mounts and other joints for numerical collision queries.</p><button class="button" data-capture-context="${field}" data-group="${esc(group.name)}">Capture full joint state</button><div class="requirements-grid">${Object.entries(values).map(([joint,value]) => reqField(joint,['groups',group.name,field,joint],value)).join("")}</div></details>`;
}
const groupPreviews = {};
function groupPolicyControls(group, recipe) {
  const policy=recipe.probe || {};
  const mode=policy.mode || (recipe.waypoints?.length ? 'manual' : recipe.allow_sessions ? 'hold' : 'read');
  return `<label class="field">Test mode<select data-group-mode="${esc(group.name)}">${[['read','Read-only'],['hold','Hold position'],['small','Small movements'],['manual','Named poses (advanced)']].map(([value,label])=>`<option value="${value}" ${mode===value?'selected':''}>${label}</option>`).join('')}</select></label>${['hold','small'].includes(mode) ? reqToggle('This group may use controller sessions in the current environment',['groups',group.name,'probe','reviewed'],policy.reviewed || (!recipe.probe && recipe.allow_sessions))+'<p class="small">Confirm support, balance and controller conditions for this group. Joint limits alone do not establish collision-free or stable movement.</p>' : ''}${mode==='small' ? `<div class="requirements-grid">${reqField('Angular movement budget · radians',['groups',group.name,'probe','angular_step'],policy.angular_step ?? .05)}${reqField('Linear movement budget · metres',['groups',group.name,'probe','linear_step'],policy.linear_step ?? .005)}</div><p class="small">At run time, measured positions generate one-joint probes with a return to baseline between them. The exact recipe is recorded with each result.</p><button class="button" data-preview-group="${esc(group.name)}">Preview generated movements</button>${groupPreviews[group.name] ? `<p class="small">${groupPreviews[group.name].tested_joints.length} joints · ${groupPreviews[group.name].waypoints.length} targets · numerical joint limits checked. Collision and balance eligibility must be reviewed.</p><details><summary>Generated targets</summary><pre class="technical">${esc(JSON.stringify(groupPreviews[group.name].waypoints,null,2))}</pre></details>` : ""}` : ''}`;
}
function groupRequirementCard(group) {
  const g = settingsDraft.groups?.[group.name] || {};
  const path = ['groups',group.name];
  const motion = group.control_modes?.includes('trajectory');
  const summary = g.probe ? `${g.probe.mode} mode · ${g.probe.reviewed ? "Group eligibility confirmed" : "Group eligibility not confirmed"}${g.probe.mode==='small' ? " · targets generated at run time" : ""}` : `${g.allow_sessions || g.waypoints?.length ? "Existing manual configuration" : "Read-only by default"} · ${g.waypoints?.length || 0} named targets`;
  const fields = [['dt','Seconds per waypoint',3],['settle','Settling time · seconds',.5],['tolerance','Joint error tolerance · native units',.01],['restore_timeout','Restoration timeout · seconds',10],['max_check_seconds','Collision-query time budget · seconds',null]];
  const tool = group.tool;
  return `<section class="panel settings-card requirements-card" id="${esc(groupRequirementId(group.name))}"><div class="body"><h2>${esc(group.name)} · ${group.joints.length} joints</h2><p class="settings-summary">${esc(summary)}</p>${g.fixture_provenance ? `<p class="small">Reviewed ${esc(g.fixture_provenance.oracle)} fixtures · ${esc(g.fixture_provenance.candidate_ids.length)} candidates · ${esc(g.fixture_provenance.reviewed_at)}</p>` : ""}${motion ? groupPolicyControls(group,g) : "<p>Read-only group: no trajectory controller declared.</p>"}<details data-open-key="${esc(group.name)}"><summary>Advanced fixtures and measurements</summary><p>${motion ? `Trajectory controller: ${esc(motion)}.` : "No trajectory controller is declared; motion-session checks are not applicable."} ${group.tip ? `Cartesian tip: ${esc(group.tip)}.` : "Joint-space group; Cartesian tip checks are not applicable."}</p><button class="button" data-read-group="${esc(group.name)}">Read current state</button><p class="small">${groupReadings[group.name] ? "Measured positions and native units loaded. Capture does not move the robot." : "Read or capture state to see current positions, units and live limits."}</p>${motion ? reqToggle('Enable controller-session checks',[...path,'allow_sessions'],g.allow_sessions || !!g.waypoints?.length) : ""}<div class="requirements-grid">${fields.map(([k,label,value]) => reqField(label,[...path,k],g[k] ?? (k==='tolerance' && g.probe?.mode==='small' ? .001 : value),'number',true)).join("")}</div>${motion ? poseRequirements(group,'waypoints','Trajectory targets (at least two)') : ""}${poseRequirements(group,'clear_configurations','Known clear configurations')}${poseRequirements(group,'collision_configurations','Known colliding configurations')}${stateRequirement(group,'clear_state','Joint context for clear configurations')}${stateRequirement(group,'collision_state','Joint context for collision configurations')}${tool?.joint != null && tool.open != null && tool.closed != null ? `<details class="fixture-section" data-open-key="${esc(group.name+'tool')}"><summary>Tool motion and obstruction fixture</summary><p class="small">${esc(tool.type)} · ${esc(tool.joint)} · endpoints ${tool.open} / ${tool.closed}, declared by the robot profile.</p><label class="command-toggle"><input type="checkbox" data-enable-tool="${esc(group.name)}" ${g.tool ? 'checked' : ''} />Enable tool motion tests</label>${g.tool ? `<div class="requirements-grid">${reqField('Settling time · seconds',[...path,'tool','settle'],g.tool.settle ?? 1)}${reqField('Endpoint tolerance · native units',[...path,'tool','tolerance'],g.tool.tolerance ?? .01)}${reqField('Timeout · seconds',[...path,'tool','timeout'],g.tool.timeout ?? 15)}${reqField('Reviewed obstruction target',[...path,'tool','blocked_target'],g.tool.blocked_target,'number',true)}</div><p class="small">An obstruction target needs known object contact or an explicit simulator stall fixture. It is not a target past a mechanical hard stop.</p>${reqCommand('Simulator stall command',[...path,'tool','stall_command'],g.tool.stall_command)}${reqCommand('Simulator stall restore command',[...path,'tool','stall_restore_command'],g.tool.stall_restore_command)}` : ''}</details>` : ''}</details></div></section>`;
}
function navigationPoseList(field, title) {
  const poses=settingsDraft.navigation?.[field] || [];
  return `<details class="fixture-section" data-open-key="${field}"><summary>${title} · ${poses.length}</summary>${poses.map((pose,i) => `<div class="requirements-grid fixture-pose">${['x','y','z','yaw','frame'].map((key) => reqField(key,['navigation',field,i,key],pose[key],key==='frame'?'text':'number')).join('')}<button class="textbutton" data-remove-nav="${i}" data-nav-field="${field}">Remove goal</button></div>`).join('')}<button class="button" data-add-nav="${field}">Add goal</button><p class="small">${field==='unreachable_goals' ? 'Use an independently known unreachable target in this environment. The harness does not invent a failure goal.' : 'Manual goals take precedence over automatic map selection. Supply coordinates and their explicit frame.'}</p></details>`;
}
function actionRequirementCard(name,spec) {
  const a=settingsDraft.actions?.[name];
  const args=(kind) => Object.entries(spec.parameters || {}).map(([key,p]) => {
    const value=a?.[kind]?.[key] ?? p.default;
    return p.type==='bool' ? reqToggle(key,['actions',name,kind,key],value) : reqField(`${key}${p.min!=null || p.max!=null ? ` · bounds ${p.min ?? '−∞'} to ${p.max ?? '∞'}` : ''}`,['actions',name,kind,key],value,p.type==='string'?'text':'number',true);
  }).join('');
  return `<section class="panel settings-card" id="requirements-action-${esc(encodeURIComponent(name))}"><div class="body"><h2>Action · ${esc(name)}</h2><p>${spec.is_motion ? 'This action moves the robot.' : 'This is a configured custom command.'} Verification needs an observable SDK feedback change.</p><label class="command-toggle"><input type="checkbox" data-enable-action="${esc(name)}" ${a ? 'checked' : ''} />Configure execution test</label>${a ? `<div class="requirements-grid">${args('arguments')}${reqField('SDK feedback path',['actions',name,'feedback','path'],a.feedback?.path,'text')}${reqField('Expected measured value',['actions',name,'feedback','expected'],a.feedback?.expected)}${reqField('Feedback tolerance',['actions',name,'feedback','tolerance'],a.feedback?.tolerance ?? .01)}${reqField('Timeout · seconds',['actions',name,'timeout'],a.timeout ?? 10)}</div><p class="small">Examples: sensors.motor.temperature, arms.group.joint_positions.joint_name, velocity.linear.x. A command response alone does not prove the effect.</p><label class="command-toggle"><input type="checkbox" data-restore-action="${esc(name)}" ${a.restore_arguments ? 'checked' : ''} />Restore using these arguments</label>${a.restore_arguments ? `<div class="requirements-grid">${args('restore_arguments')}</div>` : ''}` : ''}</div></section>`;
}
function renderRequirements() {
  const meta=settingsMeta;
  if (!meta) { $('test-requirements').innerHTML='<p>Inspect or reload the running robot to configure its requirements.</p>'; return; }
  const open = new Set([...$('test-requirements').querySelectorAll('details[open][data-open-key]')].map((el)=>el.dataset.openKey));
  const watch=settingsDraft.teleop?.watch_stop;
  const description=settingsDraft.description_change;
  const tolerances=settingsDraft.tolerances || {};
  const caps=new Set(meta.capabilities);
  let html=preparationCard()+`${meta.stale_model ? `<section class="panel settings-card"><div class="body"><h2>Robot model changed</h2><p>Saved fixtures refer to a different model and are excluded from this draft. Read-only defaults are shown. Review group eligibility again before enabling commands. The previous settings remain at ${esc(meta.saved_settings_path)}; saving creates a backup.</p></div></section>` : ""}<div id="requirements-groups"><h2>Joint groups</h2><p>Choose a mode for each group. Small movements generate targets from live joint feedback; manual poses are optional advanced fixtures.</p>${meta.groups.map(groupRequirementCard).join('') || '<p class="small">No joint groups advertised.</p>'}</div>`;
  if (caps.has('watches') && settingsDraft.teleop) html+=`<section class="panel settings-card" id="requirements-watch"><div class="body"><h2>Watch-triggered stopping</h2><label class="command-toggle"><input type="checkbox" data-enable-watch ${watch ? 'checked' : ''} />Configure a stop condition</label><p class="small">Choose feedback that becomes true after the configured drive begins.</p>${watch ? `<div class="requirements-grid">${reqField('SDK state path',['teleop','watch_stop','path'],watch.path,'text')}<label class="field">Condition<select id="watch-condition"><option value="above" ${'above' in watch?'selected':''}>Above</option><option value="below" ${'below' in watch?'selected':''}>Below</option></select></label>${reqField('Threshold',['teleop','watch_stop','above' in watch?'above':'below'],watch.above ?? watch.below)}</div>` : ''}</div></section>`;
  if (settingsDraft.navigation) html+=`<section class="panel settings-card" id="requirements-navigation"><div class="body"><h2>Navigation test targets</h2>${navigationPoseList('goals','Reviewed navigation goals')}${navigationPoseList('unreachable_goals','Known unreachable goals')}</div></section>`;
  if (['slam','places','mobility'].every((c)=>caps.has(c))) html+=`<section class="panel settings-card" id="requirements-mapping"><div class="body"><h2>Map save and localization</h2>${reqToggle('Enable save → localize → navigate → restore',['mapping','save_localize'],settingsDraft.mapping?.save_localize)}<p class="small">Requires mapping mode, navigation goals and an agent that supports preserving the mapping graph. The test restores the previous map state and removes its temporary map/place.</p></div></section>`;
  html+=`<section class="panel settings-card" id="requirements-description"><div class="body"><h2>Model change and restoration</h2><label class="command-toggle"><input type="checkbox" data-enable-description ${description ? 'checked' : ''} />Configure model-change test</label><p class="small">Commands run on the test-environment host. Use an explicit container or remote wrapper when the publisher runs elsewhere. Both change and restore are required.</p>${description ? reqCommand('Change command',['description_change','command'],description.command)+reqCommand('Restore command',['description_change','restore_command'],description.restore_command)+reqField('Timeout · seconds',['description_change','timeout'],description.timeout ?? 60) : ''}</div></section>`;
  html+=`<section class="panel settings-card" id="requirements-artifacts"><div class="body"><h2>Recorded model and reach map</h2>${reqField('Snapshot search directory on the test host',['artifacts','snapshot_dir'],settingsDraft.artifacts?.snapshot_dir,'text',true)}<p class="small">${meta.snapshot ? `Matching snapshot: ${esc(meta.snapshot)} · reach map ${meta.reach_map ? 'recorded' : 'not recorded'}.` : 'No snapshot matching the live robot description was found.'}</p><details data-open-key="artifact-help"><summary>How to record the geometry</summary><p>Run onboarding in the robot’s ROS environment, where its mesh packages are available, then expose the output folder to this test host.</p><pre class="technical">ros2 run ringfour_agent onboard --profile /path/to/profile.yaml --out /path/to/profiles --skip-reach</pre><p class="small">Omit --skip-reach and select --group for an applicable Cartesian reach map. Joint-space groups do not need a Cartesian reach map.</p></details></div></section>`;
  html+=`<section class="panel settings-card" id="requirements-tolerances"><div class="body"><h2>Model and joint measurement allowances</h2><details data-open-key="measurement"><summary>Advanced measurement settings</summary><div class="requirements-grid">${reqField('FK position tolerance · metres',['tolerances','fk_position'],tolerances.fk_position ?? .00001)}${reqField('FK quaternion error',['tolerances','fk_quaternion'],tolerances.fk_quaternion ?? .00001)}${[...new Set(meta.groups.flatMap((g)=>g.joints))].map((joint)=>reqField(`${joint} · limit allowance in native units`,['tolerances','joint_limits',joint],tolerances.joint_limits?.[joint],'number',true)).join('')}</div><p class="small">Use measured uncertainty or calibrated compliance. These allowances do not extend legal command targets.</p></details></div></section>`;
  html+=Object.entries(meta.actions || {}).map(([name,spec])=>actionRequirementCard(name,spec)).join('');
  html+=`<section class="panel settings-card" id="requirements-connection"><div class="body"><h2>Additional transport</h2>${reqField('WebSocket address when the primary connection is Unix',['connection','tcp_uri'],settingsDraft.connection?.tcp_uri,'text',true)}<p class="small">Optional: ws://host:port. The network check verifies it reaches the same robot as the primary connection.</p></div></section>`;
  html+=`<section class="panel settings-card" id="requirements-runtime"><div class="body"><h2>Runtime prerequisites</h2>${reqField("Per-test wall-clock timeout · seconds",["runtime","test_timeout"],settingsDraft.runtime?.test_timeout ?? 180,"number",true)}<p>Independent robot YAML: ${meta.profile_selected ? 'selected' : 'select the effective profile in Robot profile'}.</p><p>Kinematics backend: ${meta.kinematics_installed ? 'installed' : 'missing; install the SDK kinematics extra on the test host'}.</p>${!meta.kinematics_installed ? '<pre class="technical">pip install -e "ringfour-sdk[kinematics]"</pre>' : ''}<p class="small">Live model: ${esc(meta.description_sha || 'not available')}. Timestamped joint/TF feedback and mapping graph preservation require agent support; configuration cannot substitute for these APIs.</p>${meta.separate_descriptions?.length ? `<p class="small">Separate group descriptions: ${esc(meta.separate_descriptions.join(', '))}. Current SDK support may limit Cartesian checks for these groups.</p>` : ''}</div></section>`;
  $('test-requirements').innerHTML=html;
  renderPreparation();
  $('test-requirements').querySelectorAll('details[data-open-key]').forEach((el)=>el.open=open.has(el.dataset.openKey));
}
function installRequirementHandlers() {
  $('test-requirements').addEventListener('input',(e)=>{
    const input=e.target.closest('[data-requirement-path]'); if (!input) return;
    const path=JSON.parse(input.dataset.requirementPath);
    const empty=input.value==='';
    const value=input.dataset.argumentList ? input.value.split(/\r?\n/).filter((s)=>s.length) : input.type==='checkbox' ? input.checked : input.type==='number' ? (empty ? null : Number(input.value)) : input.value;
    if (path[0]==='groups') delete groupPreviews[path[1]];
    reqSet(path,value,(empty && input.dataset.optional==='true') || (input.dataset.argumentList && !value.length && path[0]==='groups'));
  });
  $('test-requirements').addEventListener('change',(e)=>{
    const input=e.target;
    if (input.dataset.groupMode) {
      const recipe=(settingsDraft.groups ||= {})[input.dataset.groupMode] ||= {};
      delete groupPreviews[input.dataset.groupMode];
      const previous=recipe.probe || {};
      recipe.probe={...previous,mode:input.value,reviewed:previous.reviewed === true};
      if (input.value==='small' && recipe.tolerance==null) recipe.tolerance=.001;
    } else if (input.hasAttribute('data-enable-watch')) {
      if (input.checked) settingsDraft.teleop.watch_stop={path:'velocity.linear.x',above:.05}; else delete settingsDraft.teleop.watch_stop;
    } else if (input.id==='watch-condition') {
      const watch=settingsDraft.teleop.watch_stop, value=watch.above ?? watch.below;
      delete watch.above;delete watch.below;watch[input.value]=value;
    } else if (input.hasAttribute('data-enable-description')) {
      if (input.checked) settingsDraft.description_change={command:[],restore_command:[],timeout:60}; else delete settingsDraft.description_change;
    } else if (input.dataset.enableTool) {
      const group=settingsDraft.groups ||= {}, recipe=group[input.dataset.enableTool] ||= {};
      if (input.checked) recipe.tool={settle:1,tolerance:.01,timeout:15}; else delete recipe.tool;
    } else if (input.dataset.enableAction) {
      const actions=settingsDraft.actions ||= {};
      if (input.checked) {
        const parameters=settingsMeta.actions[input.dataset.enableAction].parameters || {};
        const args=Object.fromEntries(Object.entries(parameters).filter(([k,p])=>p.default!=null || p.type==='bool').map(([k,p])=>[k,p.default ?? false]));
        actions[input.dataset.enableAction]={arguments:args,feedback:{path:'',expected:null,tolerance:.01},timeout:10};
      } else delete actions[input.dataset.enableAction];
    } else if (input.dataset.restoreAction) {
      const action=settingsDraft.actions[input.dataset.restoreAction];
      if (input.checked) action.restore_arguments=structuredClone(action.arguments); else delete action.restore_arguments;
    } else return;
    reqDirty();renderRequirements();
  });
  $('test-requirements').addEventListener('click',(e)=>{
    const button=e.target.closest('button');if (!button) return;
    settingsAction(async()=>{
      const name=button.dataset.group || button.dataset.readGroup || button.dataset.previewGroup;
      const group=name ? settingsMeta.groups.find((g)=>g.name===name) : null;
      let recipe;
      if (group) recipe=(settingsDraft.groups ||= {})[name] ||= {};
      if (button.dataset.previewGroup) {
        button.disabled=true;
        try {
          groupPreviews[name]=await api('/api/test-settings/group-preview',{uri:$('uri').value.trim(),profile:$('profile').value,group:name,recipe});
          renderRequirements();
        } finally { button.disabled=false; }
      } else if (button.dataset.readGroup || button.dataset.capturePose || button.dataset.captureContext) {
        button.disabled=true;
        try {
          const state=await api('/api/test-settings/group-state',{uri:$('uri').value.trim(),profile:$('profile').value,group:name});
          groupReadings[name]=state;
          if (button.dataset.capturePose) {
            const field=button.dataset.capturePose;
            if (field!=='waypoints') delete recipe.fixture_provenance;
            const pose=Object.fromEntries(group.joints.map((j)=>[j,state.joints.find((x)=>x.name===j).position]));
            const poses=recipe[field] ||= [];
            if (field==='waypoints' && !poses.length) poses.push(structuredClone(pose));
            poses.push(pose);
            reqDirty();
          }
          if (button.dataset.captureContext) { delete recipe.fixture_provenance;recipe[button.dataset.captureContext]=state.positions;reqDirty(); }
          renderRequirements();
        } finally { button.disabled=false; }
      } else if (button.dataset.addPose) {
        if (button.dataset.addPose!=='waypoints') delete recipe.fixture_provenance;
        (recipe[button.dataset.addPose] ||= []).push({});reqDirty();renderRequirements();
      } else if (button.dataset.removePose !== undefined) {
        const field=button.dataset.poseField;if (field!=='waypoints') delete recipe.fixture_provenance;recipe[field].splice(Number(button.dataset.removePose),1);
        if (!recipe[field].length) delete recipe[field];reqDirty();renderRequirements();
      } else if (button.dataset.addNav) {
        (settingsDraft.navigation[button.dataset.addNav] ||= []).push({x:null,y:null,z:0,yaw:0,frame:settingsMeta.frames.map || ''});reqDirty();renderRequirements();
      } else if (button.dataset.removeNav !== undefined) {
        const field=button.dataset.navField;settingsDraft.navigation[field].splice(Number(button.dataset.removeNav),1);
        if (!settingsDraft.navigation[field].length) delete settingsDraft.navigation[field];reqDirty();renderRequirements();
      }
    });
  });
}
